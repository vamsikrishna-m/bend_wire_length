"""
FIXED: Professional STEP File Wire Analyzer
Properly consolidates arc segments into single bends
Handles ANY number of bends - NO HARDCODING
"""

import numpy as np
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import math

# OpenCASCADE imports
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
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
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    def __repr__(self):
        return f"Point3D({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"
    
    @staticmethod
    def from_gp_Pnt(pnt: gp_Pnt) -> 'Point3D':
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
        self.sub_edges = []  # Track multiple edges that form this segment
        
    def __repr__(self):
        if self.type == 'BEND':
            return f"Bend(R={self.radius:.2f}mm, θ={self.angle_deg:.1f}°, L={self.length:.2f}mm)"
        else:
            return f"Straight(L={self.length:.2f}mm)"


class STEPWireAnalyzer:
    """
    Professional STEP file analyzer
    Consolidates multiple arc edges into single bends
    """
    
    def __init__(self, step_file_path: str):
        self.step_file = Path(step_file_path)
        self.shape = None
        self.edges = []
        self.raw_segments = []  # Before consolidation
        self.segments = []  # After consolidation
        
        # Results
        self.total_length = 0.0
        self.bend_count = 0
        self.straight_count = 0
        self.centerline_points = []
        
    def load_step_file(self) -> bool:
        """Load STEP file using pythonocc-core"""
        print(f"Loading STEP file: {self.step_file.name}")
        
        reader = STEPControl_Reader()
        status = reader.ReadFile(str(self.step_file))
        
        if status != IFSelect_RetDone:
            raise Exception(f"Error reading STEP file: {self.step_file}")
        
        print(f"✓ File read successfully")
        
        nb_roots = reader.NbRootsForTransfer()
        print(f"  Roots to transfer: {nb_roots}")
        
        if nb_roots == 0:
            raise Exception("No transferable roots found in STEP file")
        
        for i in range(1, nb_roots + 1):
            ok = reader.TransferRoot(i)
            print(f"  Root {i} transferred: {ok}")
        
        nb_shapes = reader.NbShapes()
        print(f"  Resulting shapes: {nb_shapes}")
        
        if nb_shapes == 0:
            raise Exception("No shapes were transferred")
        
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
        
        topo = TopologyExplorer(self.shape)
        
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
        
        if n_solids > 0:
            print("→ Geometry Type: SOLID (3D tube/pipe)")
        elif n_wires > 0 or n_edges > 0:
            print("→ Geometry Type: WIRE/CURVE")
        
        print()
    
    def extract_edges(self):
        """Extract all edges from the shape"""
        print("=" * 70)
        print("EXTRACTING EDGES")
        print("=" * 70)
        
        topo = TopologyExplorer(self.shape)
        self.edges = list(topo.edges())
        
        print(f"✓ Extracted {len(self.edges)} raw edges")
        print()
        
        return self.edges
    
    def analyze_edge(self, edge: TopoDS_Edge, edge_index: int) -> Optional[WireSegment]:
        """Analyze a single edge"""
        
        try:
            curve_adaptor = BRepAdaptor_Curve(edge)
            curve_type = curve_adaptor.GetType()
            
            first_param = curve_adaptor.FirstParameter()
            last_param = curve_adaptor.LastParameter()
            
            start_pnt = curve_adaptor.Value(first_param)
            end_pnt = curve_adaptor.Value(last_param)
            
            start = Point3D.from_gp_Pnt(start_pnt)
            end = Point3D.from_gp_Pnt(end_pnt)
            
            # Calculate edge length
            if HAS_GCPNTS:
                try:
                    edge_length = GCPnts_AbscissaPoint.Length_s(curve_adaptor)
                except:
                    edge_length = start.distance_to(end)
            else:
                edge_length = start.distance_to(end)
            
            if curve_type == GeomAbs_Line:
                segment = WireSegment('STRAIGHT', start, end)
                segment.length = edge_length
                segment.edge_index = edge_index
                return segment
                
            elif curve_type == GeomAbs_Circle:
                segment = WireSegment('BEND', start, end)
                segment.length = edge_length
                segment.edge_index = edge_index
                
                circle = curve_adaptor.Circle()
                radius = circle.Radius()
                center_pnt = circle.Location()
                
                segment.radius = radius
                segment.center = Point3D.from_gp_Pnt(center_pnt)
                
                # Calculate angle
                v1 = start.to_array() - segment.center.to_array()
                v2 = end.to_array() - segment.center.to_array()
                
                v1_len = np.linalg.norm(v1)
                v2_len = np.linalg.norm(v2)
                
                if v1_len > 1e-6 and v2_len > 1e-6:
                    v1_norm = v1 / v1_len
                    v2_norm = v2 / v2_len
                    
                    cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
                    angle_rad = np.arccos(cos_angle)
                    segment.angle_deg = np.degrees(angle_rad)
                
                return segment
            
            else:
                segment = WireSegment('STRAIGHT', start, end)
                segment.length = edge_length
                segment.edge_index = edge_index
                return segment
                
        except Exception as e:
            print(f"  Warning: Could not analyze edge {edge_index}: {e}")
            return None
    
    def consolidate_segments(self):
        """
        CRITICAL: Consolidate consecutive arc segments into single bends
        This is where we fix the "too many segments" problem
        """
        print("=" * 70)
        print("CONSOLIDATING SEGMENTS")
        print("=" * 70)
        
        if not self.raw_segments:
            print("No segments to consolidate")
            return []
        
        consolidated = []
        i = 0
        
        while i < len(self.raw_segments):
            current = self.raw_segments[i]
            
            if current.type == 'STRAIGHT':
                # Check if next segments are also straight and collinear
                merged_straight = self._merge_collinear_straights(i)
                consolidated.append(merged_straight)
                i += len(merged_straight.sub_edges)
                
            elif current.type == 'BEND':
                # Check if next segments are also arcs with same center/radius
                merged_bend = self._merge_concentric_arcs(i)
                consolidated.append(merged_bend)
                i += len(merged_bend.sub_edges)
            
            else:
                consolidated.append(current)
                i += 1
        
        print(f"✓ Consolidated {len(self.raw_segments)} raw edges into {len(consolidated)} segments")
        print(f"  Straight: {sum(1 for s in consolidated if s.type == 'STRAIGHT')}")
        print(f"  Bends: {sum(1 for s in consolidated if s.type == 'BEND')}")
        print()
        
        return consolidated
    
    def _merge_collinear_straights(self, start_idx):
        """Merge consecutive collinear straight segments"""
        segments_to_merge = [self.raw_segments[start_idx]]
        
        i = start_idx + 1
        while i < len(self.raw_segments):
            next_seg = self.raw_segments[i]
            
            if next_seg.type != 'STRAIGHT':
                break
            
            # Check if collinear (same direction within tolerance)
            prev = segments_to_merge[-1]
            
            # Direction vectors
            dir1 = (prev.end.to_array() - prev.start.to_array())
            dir1 = dir1 / np.linalg.norm(dir1)
            
            dir2 = (next_seg.end.to_array() - next_seg.start.to_array())
            dir2 = dir2 / np.linalg.norm(dir2)
            
            # Check if parallel (dot product close to 1)
            if abs(np.dot(dir1, dir2) - 1.0) < 0.01:
                segments_to_merge.append(next_seg)
                i += 1
            else:
                break
        
        # Create merged segment
        merged = WireSegment('STRAIGHT', segments_to_merge[0].start, segments_to_merge[-1].end)
        merged.length = sum(s.length for s in segments_to_merge)
        merged.sub_edges = segments_to_merge
        
        return merged
    
    def _merge_concentric_arcs(self, start_idx):
        """Merge consecutive arc segments with same center and radius"""
        segments_to_merge = [self.raw_segments[start_idx]]
        
        first_seg = self.raw_segments[start_idx]
        if not first_seg.center or not first_seg.radius:
            return first_seg
        
        i = start_idx + 1
        while i < len(self.raw_segments):
            next_seg = self.raw_segments[i]
            
            if next_seg.type != 'BEND':
                break
            
            if not next_seg.center or not next_seg.radius:
                break
            
            # Check if same center and radius
            center_dist = first_seg.center.distance_to(next_seg.center)
            radius_diff = abs(first_seg.radius - next_seg.radius)
            
            if center_dist < 0.1 and radius_diff < 0.1:
                segments_to_merge.append(next_seg)
                i += 1
            else:
                break
        
        # Create merged bend
        merged = WireSegment('BEND', segments_to_merge[0].start, segments_to_merge[-1].end)
        merged.length = sum(s.length for s in segments_to_merge)
        merged.radius = first_seg.radius
        merged.center = first_seg.center
        merged.sub_edges = segments_to_merge
        
        # Calculate total angle
        v1 = segments_to_merge[0].start.to_array() - merged.center.to_array()
        v2 = segments_to_merge[-1].end.to_array() - merged.center.to_array()
        
        v1_norm = v1 / np.linalg.norm(v1)
        v2_norm = v2 / np.linalg.norm(v2)
        
        cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle_rad = np.arccos(cos_angle)
        merged.angle_deg = np.degrees(angle_rad)
        
        return merged
    
    def analyze_all_segments(self):
        """Analyze all edges and consolidate"""
        print("=" * 70)
        print("ANALYZING SEGMENTS")
        print("=" * 70)
        
        # First pass: analyze all raw edges
        self.raw_segments = []
        for i, edge in enumerate(self.edges):
            segment = self.analyze_edge(edge, i)
            if segment:
                self.raw_segments.append(segment)
        
        print(f"✓ Analyzed {len(self.raw_segments)} raw segments")
        
        # Second pass: consolidate
        self.segments = self.consolidate_segments()
        
        # Count final segments
        self.bend_count = sum(1 for s in self.segments if s.type == 'BEND')
        self.straight_count = sum(1 for s in self.segments if s.type == 'STRAIGHT')
        self.total_length = sum(s.length for s in self.segments)
        
        print(f"  Total length: {self.total_length:.2f} mm")
        print()
        
        return self.segments
    
    def print_segment_details(self):
        """Print detailed information"""
        print("=" * 70)
        print("SEGMENT DETAILS")
        print("=" * 70)
        print()
        
        for i, segment in enumerate(self.segments, 1):
            print(f"Segment {i}: {segment.type}")
            print(f"  Start: ({segment.start.x:.2f}, {segment.start.y:.2f}, {segment.start.z:.2f})")
            print(f"  End:   ({segment.end.x:.2f}, {segment.end.y:.2f}, {segment.end.z:.2f})")
            print(f"  Length: {segment.length:.2f} mm")
            
            if segment.sub_edges:
                print(f"  (Merged from {len(segment.sub_edges)} raw edges)")
            
            if segment.type == 'BEND':
                if segment.radius:
                    print(f"  Radius: {segment.radius:.2f} mm")
                if segment.angle_deg:
                    print(f"  Angle: {segment.angle_deg:.2f}°")
                if segment.center:
                    print(f"  Center: ({segment.center.x:.2f}, {segment.center.y:.2f}, {segment.center.z:.2f})")
            
            print()
    
    def build_centerline(self, points_per_segment: int = 20):
        """Build 3D centerline"""
        print("=" * 70)
        print("BUILDING 3D CENTERLINE")
        print("=" * 70)
        
        self.centerline_points = []
        
        for segment in self.segments:
            if segment.type == 'STRAIGHT':
                for i in range(points_per_segment):
                    t = i / (points_per_segment - 1) if points_per_segment > 1 else 0
                    point = (
                        segment.start.x + t * (segment.end.x - segment.start.x),
                        segment.start.y + t * (segment.end.y - segment.start.y),
                        segment.start.z + t * (segment.end.z - segment.start.z)
                    )
                    self.centerline_points.append(point)
                    
            elif segment.type == 'BEND' and segment.center and segment.radius:
                center = segment.center.to_array()
                start = segment.start.to_array()
                end = segment.end.to_array()
                
                v_start = start - center
                start_angle = np.arctan2(v_start[1], v_start[0])
                
                v_end = end - center
                end_angle = np.arctan2(v_end[1], v_end[0])
                
                angle_diff = end_angle - start_angle
                if angle_diff > np.pi:
                    angle_diff -= 2 * np.pi
                elif angle_diff < -np.pi:
                    angle_diff += 2 * np.pi
                
                for i in range(points_per_segment):
                    t = i / (points_per_segment - 1) if points_per_segment > 1 else 0
                    angle = start_angle + t * angle_diff
                    
                    point = (
                        center[0] + segment.radius * np.cos(angle),
                        center[1] + segment.radius * np.sin(angle),
                        center[2] + t * (end[2] - start[2])
                    )
                    self.centerline_points.append(point)
            else:
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


def analyze_step_file(step_file_path: str, output_dir: Optional[str] = None):
    """Main function to analyze STEP file"""
    print()
    print("#" * 70)
    print(f"# ANALYZING: {Path(step_file_path).name}")
    print("#" * 70)
    print()
    
    analyzer = STEPWireAnalyzer(step_file_path)
    analyzer.load_step_file()
    analyzer.analyze_topology()
    analyzer.extract_edges()
    analyzer.analyze_all_segments()
    analyzer.print_segment_details()
    analyzer.build_centerline(points_per_segment=20)
    analyzer.print_summary()
    
    return analyzer


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        step_file = sys.argv[1]
        output_dir = sys.argv[2] if len(sys.argv) > 2 else "analysis_results"
        analyzer = analyze_step_file(step_file, output_dir)
    else:
        print("Usage: python step_analyzer_final.py <step_file> [output_dir]")
