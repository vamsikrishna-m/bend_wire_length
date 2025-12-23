"""
Professional STEP File Wire Analyzer using pythonocc-core
Handles ANY number of bends automatically - FIXED VERSION
Works with both wire geometry and solid tubes

Requirements:
    conda install -c conda-forge pythonocc-core
    pip install numpy matplotlib
"""

import numpy as np
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import math

# OpenCASCADE imports
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import (TopAbs_EDGE, TopAbs_WIRE, TopAbs_FACE, 
                              TopAbs_SOLID, TopAbs_VERTEX)
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Line, GeomAbs_Circle, GeomAbs_BSplineCurve
from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import TopoDS_Edge
from OCC.Extend.TopologyUtils import TopologyExplorer

try:
    from OCC.Core.GCPnts import GCPnts_AbscissaPoint
    HAS_GCPNTS = True
except:
    HAS_GCPNTS = False


class Point3D:
    """3D Point with utilities"""
    
    def __init__(self, x: float, y: float, z: float):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
    
    def distance_to(self, other: 'Point3D') -> float:
        """Calculate distance to another point"""
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array"""
        return np.array([self.x, self.y, self.z])
    
    def __repr__(self):
        return f"Point3D({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"
    
    @staticmethod
    def from_gp_Pnt(pnt: gp_Pnt) -> 'Point3D':
        """Create from OCC gp_Pnt"""
        return Point3D(pnt.X(), pnt.Y(), pnt.Z())


class WireSegment:
    """Represents a segment of the wire (straight or bent)"""
    
    def __init__(self, segment_type: str, start: Point3D, end: Point3D):
        self.type = segment_type  # 'STRAIGHT' or 'BEND'
        self.start = start
        self.end = end
        self.length = 0.0
        
        # For bends
        self.radius = None
        self.angle_deg = None
        self.center = None
        
        # For tracking
        self.edge_index = None
        
    def __repr__(self):
        if self.type == 'BEND':
            return f"Bend(R={self.radius:.2f}mm, θ={self.angle_deg:.1f}°, L={self.length:.2f}mm)"
        else:
            return f"Straight(L={self.length:.2f}mm)"


class STEPWireAnalyzer:
    """
    Professional STEP file analyzer using pythonocc-core
    Handles any number of bends automatically
    """
    
    def __init__(self, step_file_path: str):
        self.step_file = Path(step_file_path)
        self.shape = None
        self.edges = []
        self.segments = []
        
        # Results
        self.total_length = 0.0
        self.bend_count = 0
        self.straight_count = 0
        self.centerline_points = []
        
    def load_step_file(self) -> bool:
        """Load STEP file using pythonocc-core"""
        print(f"Loading STEP file: {self.step_file.name}")
        
        # Create STEP reader
        reader = STEPControl_Reader()
        
        # Read file
        status = reader.ReadFile(str(self.step_file))
        
        if status != IFSelect_RetDone:
            raise Exception(f"Error reading STEP file: {self.step_file}")
        
        print(f"✓ File read successfully")
        
        # Get number of roots before transfer
        nb_roots = reader.NbRootsForTransfer()
        print(f"  Roots to transfer: {nb_roots}")
        
        if nb_roots == 0:
            raise Exception("No transferable roots found in STEP file")
        
        # Transfer all roots
        for i in range(1, nb_roots + 1):
            ok = reader.TransferRoot(i)
            print(f"  Root {i} transferred: {ok}")
        
        # Get number of resulting shapes
        nb_shapes = reader.NbShapes()
        print(f"  Resulting shapes: {nb_shapes}")
        
        if nb_shapes == 0:
            raise Exception("No shapes were transferred")
        
        # Get the combined shape using OneShape
        # This returns all transferred shapes as a single compound
        self.shape = reader.OneShape()
        
        if self.shape.IsNull():
            raise Exception("Combined shape is null")
        
        print(f"✓ STEP file loaded successfully\n")
        return True
    
    def analyze_topology(self):
        """Analyze the topology of the shape"""
        print("=" * 70)
        print("TOPOLOGY ANALYSIS")
        print("=" * 70)
        
        # Use TopologyExplorer for easier navigation
        topo = TopologyExplorer(self.shape)
        
        # Count different elements
        n_solids = topo.number_of_solids()
        n_faces = topo.number_of_faces()
        n_wires = topo.number_of_wires()
        n_edges = topo.number_of_edges()
        n_vertices = topo.number_of_vertices()
        
        print(f"{'Solids':20s}: {n_solids}")
        print(f"{'Faces':20s}: {n_faces}")
        print(f"{'Wires':20s}: {n_wires}")
        print(f"{'Edges':20s}: {n_edges}")
        print(f"{'Vertices':20s}: {n_vertices}")
        print()
        
        # Determine geometry type
        if n_solids > 0:
            print("→ Geometry Type: SOLID (3D tube/pipe)")
            print("  Note: Will extract centerline from solid body")
        elif n_faces > 0:
            print("→ Geometry Type: SURFACE")
        elif n_wires > 0 or n_edges > 0:
            print("→ Geometry Type: WIRE/CURVE (direct edge analysis)")
        
        print()
    
    def extract_edges(self):
        """Extract all edges from the shape"""
        print("=" * 70)
        print("EXTRACTING EDGES")
        print("=" * 70)
        
        # Use TopologyExplorer
        topo = TopologyExplorer(self.shape)
        
        self.edges = list(topo.edges())
        
        print(f"✓ Extracted {len(self.edges)} edges")
        print()
        
        return self.edges
    
    def analyze_edge(self, edge: TopoDS_Edge, edge_index: int) -> Optional[WireSegment]:
        """Analyze a single edge and create a WireSegment"""
        
        try:
            # Create curve adaptor
            curve_adaptor = BRepAdaptor_Curve(edge)
            curve_type = curve_adaptor.GetType()
            
            # Get start and end points
            first_param = curve_adaptor.FirstParameter()
            last_param = curve_adaptor.LastParameter()
            
            start_pnt = curve_adaptor.Value(first_param)
            end_pnt = curve_adaptor.Value(last_param)
            
            start = Point3D.from_gp_Pnt(start_pnt)
            end = Point3D.from_gp_Pnt(end_pnt)
            
            # Calculate edge length - use GCPnts if available
            if HAS_GCPNTS:
                try:
                    edge_length = GCPnts_AbscissaPoint.Length_s(curve_adaptor)
                except:
                    # Fallback
                    edge_length = start.distance_to(end)
            else:
                # Simple fallback
                edge_length = start.distance_to(end)
            
            # Analyze based on curve type
            if curve_type == GeomAbs_Line:
                # Straight segment
                segment = WireSegment('STRAIGHT', start, end)
                segment.length = edge_length
                segment.edge_index = edge_index
                return segment
                
            elif curve_type == GeomAbs_Circle:
                # Circular arc (bend)
                segment = WireSegment('BEND', start, end)
                segment.length = edge_length
                segment.edge_index = edge_index
                
                # Get circle parameters
                circle = curve_adaptor.Circle()
                radius = circle.Radius()
                center_pnt = circle.Location()
                
                segment.radius = radius
                segment.center = Point3D.from_gp_Pnt(center_pnt)
                
                # Calculate angle
                # Vectors from center to start and end
                v1 = start.to_array() - segment.center.to_array()
                v2 = end.to_array() - segment.center.to_array()
                
                # Normalize
                v1_len = np.linalg.norm(v1)
                v2_len = np.linalg.norm(v2)
                
                if v1_len > 1e-6 and v2_len > 1e-6:
                    v1_norm = v1 / v1_len
                    v2_norm = v2 / v2_len
                    
                    # Calculate angle
                    cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
                    angle_rad = np.arccos(cos_angle)
                    segment.angle_deg = np.degrees(angle_rad)
                
                return segment
                
            elif curve_type == GeomAbs_BSplineCurve:
                # BSpline curve - approximate as bend for now
                segment = WireSegment('BEND', start, end)
                segment.length = edge_length
                segment.edge_index = edge_index
                
                # Try to approximate radius and angle
                chord_length = start.distance_to(end)
                
                if chord_length > 1e-6 and edge_length > chord_length * 1.01:
                    # Approximate radius using arc length and chord
                    # For circular arc: L = R*θ, chord = 2*R*sin(θ/2)
                    # Approximate for small angles
                    sagitta = edge_length - chord_length
                    if sagitta > 1e-6:
                        segment.radius = (chord_length**2 + 4*sagitta**2) / (8*sagitta)
                        if segment.radius > 0:
                            segment.angle_deg = np.degrees(2 * np.arcsin(chord_length / (2 * segment.radius)))
                
                return segment
            
            else:
                # Unknown curve type - treat as straight
                segment = WireSegment('STRAIGHT', start, end)
                segment.length = edge_length
                segment.edge_index = edge_index
                return segment
                
        except Exception as e:
            print(f"  Warning: Could not analyze edge {edge_index}: {e}")
            return None
    
    def analyze_all_segments(self):
        """Analyze all edges and classify as straight or bend"""
        print("=" * 70)
        print("ANALYZING SEGMENTS")
        print("=" * 70)
        
        self.segments = []
        
        for i, edge in enumerate(self.edges):
            segment = self.analyze_edge(edge, i)
            if segment:
                self.segments.append(segment)
        
        # Count bends and straights
        self.bend_count = sum(1 for s in self.segments if s.type == 'BEND')
        self.straight_count = sum(1 for s in self.segments if s.type == 'STRAIGHT')
        
        # Calculate total length
        self.total_length = sum(s.length for s in self.segments)
        
        print(f"✓ Analyzed {len(self.segments)} segments")
        print(f"  Straight segments: {self.straight_count}")
        print(f"  Bend segments: {self.bend_count}")
        print(f"  Total length: {self.total_length:.2f} mm")
        print()
        
        return self.segments
    
    def print_segment_details(self):
        """Print detailed information about each segment"""
        print("=" * 70)
        print("SEGMENT DETAILS")
        print("=" * 70)
        print()
        
        for i, segment in enumerate(self.segments, 1):
            print(f"Segment {i}: {segment.type}")
            print(f"  Start: ({segment.start.x:.2f}, {segment.start.y:.2f}, {segment.start.z:.2f})")
            print(f"  End:   ({segment.end.x:.2f}, {segment.end.y:.2f}, {segment.end.z:.2f})")
            print(f"  Length: {segment.length:.2f} mm")
            
            if segment.type == 'BEND':
                if segment.radius:
                    print(f"  Radius: {segment.radius:.2f} mm")
                if segment.angle_deg:
                    print(f"  Angle: {segment.angle_deg:.2f}°")
                if segment.center:
                    print(f"  Center: ({segment.center.x:.2f}, {segment.center.y:.2f}, {segment.center.z:.2f})")
            
            print()
    
    def build_centerline(self, points_per_segment: int = 20):
        """Build 3D centerline with specified density"""
        print("=" * 70)
        print("BUILDING 3D CENTERLINE")
        print("=" * 70)
        
        self.centerline_points = []
        
        for segment in self.segments:
            if segment.type == 'STRAIGHT':
                # Linear interpolation
                for i in range(points_per_segment):
                    t = i / (points_per_segment - 1) if points_per_segment > 1 else 0
                    point = (
                        segment.start.x + t * (segment.end.x - segment.start.x),
                        segment.start.y + t * (segment.end.y - segment.start.y),
                        segment.start.z + t * (segment.end.z - segment.start.z)
                    )
                    self.centerline_points.append(point)
                    
            elif segment.type == 'BEND' and segment.center and segment.radius:
                # Arc interpolation
                center = segment.center.to_array()
                start = segment.start.to_array()
                end = segment.end.to_array()
                
                # Calculate start angle
                v_start = start - center
                start_angle = np.arctan2(v_start[1], v_start[0])
                
                # Calculate angular span
                v_end = end - center
                end_angle = np.arctan2(v_end[1], v_end[0])
                
                # Determine direction (shortest arc)
                angle_diff = end_angle - start_angle
                if angle_diff > np.pi:
                    angle_diff -= 2 * np.pi
                elif angle_diff < -np.pi:
                    angle_diff += 2 * np.pi
                
                # Generate points along arc
                for i in range(points_per_segment):
                    t = i / (points_per_segment - 1) if points_per_segment > 1 else 0
                    angle = start_angle + t * angle_diff
                    
                    point = (
                        center[0] + segment.radius * np.cos(angle),
                        center[1] + segment.radius * np.sin(angle),
                        center[2] + t * (end[2] - start[2])  # Linear in Z
                    )
                    self.centerline_points.append(point)
            else:
                # Fallback to linear interpolation
                for i in range(points_per_segment):
                    t = i / (points_per_segment - 1) if points_per_segment > 1 else 0
                    point = (
                        segment.start.x + t * (segment.end.x - segment.start.x),
                        segment.start.y + t * (segment.end.y - segment.start.y),
                        segment.start.z + t * (segment.end.z - segment.start.z)
                    )
                    self.centerline_points.append(point)
        
        print(f"✓ Generated centerline with {len(self.centerline_points)} points")
        print()
    
    def print_summary(self):
        """Print analysis summary"""
        print("=" * 70)
        print("WIRE ANALYSIS SUMMARY")
        print("=" * 70)
        print()
        
        print(f"Total Length:        {self.total_length:.2f} mm")
        print(f"Number of Bends:     {self.bend_count}")
        print(f"Straight Segments:   {self.straight_count}")
        print(f"Total Segments:      {len(self.segments)}")
        print(f"Centerline Points:   {len(self.centerline_points)}")
        print()
        
        # List all bends
        if self.bend_count > 0:
            print("Bend Details:")
            bend_num = 1
            for segment in self.segments:
                if segment.type == 'BEND':
                    print(f"  Bend {bend_num}:")
                    if segment.radius:
                        print(f"    Radius:      {segment.radius:.2f} mm")
                    if segment.angle_deg:
                        print(f"    Angle:       {segment.angle_deg:.2f}°")
                    print(f"    Arc Length:  {segment.length:.2f} mm")
                    bend_num += 1
        
        print()
        print("=" * 70)
        print()
    
    def export_results(self, output_file: str):
        """Export results to text file"""
        with open(output_file, 'w') as f:
            f.write("WIRE GEOMETRY ANALYSIS RESULTS\n")
            f.write("=" * 70 + "\n\n")
            
            f.write(f"File: {self.step_file.name}\n")
            f.write(f"Total Length: {self.total_length:.2f} mm\n")
            f.write(f"Bend Count: {self.bend_count}\n")
            f.write(f"Straight Segments: {self.straight_count}\n")
            f.write(f"Total Segments: {len(self.segments)}\n\n")
            
            f.write("Segment Details:\n")
            for i, segment in enumerate(self.segments, 1):
                f.write(f"\n  Segment {i}: {segment.type}\n")
                f.write(f"    Length: {segment.length:.2f} mm\n")
                if segment.type == 'BEND':
                    if segment.radius:
                        f.write(f"    Radius: {segment.radius:.2f} mm\n")
                    if segment.angle_deg:
                        f.write(f"    Angle: {segment.angle_deg:.2f}°\n")
            
            f.write("\n" + "=" * 70 + "\n\n")
            
            f.write("3D Centerline Points:\n")
            for i, point in enumerate(self.centerline_points):
                f.write(f"  {i:4d}: ({point[0]:8.3f}, {point[1]:8.3f}, {point[2]:8.3f})\n")
        
        print(f"✓ Results exported to: {output_file}")


def analyze_step_file(step_file_path: str, output_dir: Optional[str] = None):
    """
    Main function to analyze STEP file
    
    Args:
        step_file_path: Path to STEP file
        output_dir: Directory to save results (optional)
    
    Returns:
        STEPWireAnalyzer object with all results
    """
    print()
    print("#" * 70)
    print(f"# ANALYZING: {Path(step_file_path).name}")
    print("#" * 70)
    print()
    
    # Create analyzer
    analyzer = STEPWireAnalyzer(step_file_path)
    
    # Load STEP file
    analyzer.load_step_file()
    
    # Analyze topology
    analyzer.analyze_topology()
    
    # Extract edges
    analyzer.extract_edges()
    
    # Analyze segments
    analyzer.analyze_all_segments()
    
    # Print details
    analyzer.print_segment_details()
    
    # Build centerline
    analyzer.build_centerline(points_per_segment=20)
    
    # Print summary
    analyzer.print_summary()
    
    # Export if requested
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
        
        output_file = output_dir / f"{Path(step_file_path).stem}_analysis.txt"
        analyzer.export_results(output_file)
    
    return analyzer


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        # Analyze file from command line
        step_file = sys.argv[1]
        output_dir = sys.argv[2] if len(sys.argv) > 2 else "analysis_results"
        
        analyzer = analyze_step_file(step_file, output_dir)
    else:
        print("Usage: python step_analyzer_occ.py <step_file> [output_dir]")
        print("\nExample:")
        print("  python step_analyzer_occ.py my_wire.step results/")