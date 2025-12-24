"""
ULTRA-ROBUST STEP File Wire Analyzer
- Removes duplicate edges
- Orders edges sequentially
- Consolidates arc segments
- NO HARDCODING
"""

import numpy as np
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import math
from collections import defaultdict

# OpenCASCADE imports
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Line, GeomAbs_Circle, GeomAbs_BSplineCurve
from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import TopoDS_Edge, TopoDS_Vertex
from OCC.Extend.TopologyUtils import TopologyExplorer, WireExplorer

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
    
    def __eq__(self, other):
        if not isinstance(other, Point3D):
            return False
        return self.distance_to(other) < 0.001
    
    def __hash__(self):
        return hash((round(self.x, 3), round(self.y, 3), round(self.z, 3)))
    
    @staticmethod
    def from_gp_Pnt(pnt: gp_Pnt) -> 'Point3D':
        return Point3D(pnt.X(), pnt.Y(), pnt.Z())


class WireSegment:
    """Represents a segment of the wire"""
    
    def __init__(self, segment_type: str, start: Point3D, end: Point3D):
        self.type = segment_type
        self.start = start
        self.end = end
        self.length = 0.0
        self.radius = None
        self.angle_deg = None
        self.center = None
        self.edge_index = None
        self.sub_edges = []
        
    def __repr__(self):
        if self.type == 'BEND':
            return f"Bend(R={self.radius:.2f}mm, θ={self.angle_deg:.1f}°, L={self.length:.2f}mm)"
        else:
            return f"Straight(L={self.length:.2f}mm)"


class STEPWireAnalyzer:
    """Ultra-robust STEP file analyzer"""
    
    def __init__(self, step_file_path: str):
        self.step_file = Path(step_file_path)
        self.shape = None
        self.edges = []
        self.ordered_edges = []
        self.raw_segments = []
        self.segments = []
        
        self.total_length = 0.0
        self.bend_count = 0
        self.straight_count = 0
        self.centerline_points = []
        
    def load_step_file(self) -> bool:
        """Load STEP file"""
        print(f"Loading STEP file: {self.step_file.name}")
        
        reader = STEPControl_Reader()
        status = reader.ReadFile(str(self.step_file))
        
        if status != IFSelect_RetDone:
            raise Exception(f"Error reading STEP file")
        
        print(f"✓ File read successfully")
        
        nb_roots = reader.NbRootsForTransfer()
        print(f"  Roots to transfer: {nb_roots}")
        
        if nb_roots == 0:
            raise Exception("No transferable roots")
        
        for i in range(1, nb_roots + 1):
            reader.TransferRoot(i)
        
        self.shape = reader.OneShape()
        
        if self.shape.IsNull():
            raise Exception("Shape is null")
        
        print(f"✓ STEP file loaded\n")
        return True
    
    def analyze_topology(self):
        """Analyze topology"""
        print("=" * 70)
        print("TOPOLOGY ANALYSIS")
        print("=" * 70)
        
        topo = TopologyExplorer(self.shape)
        
        n_wires = topo.number_of_wires()
        n_edges = topo.number_of_edges()
        
        print(f"{'Wires':20s}: {n_wires}")
        print(f"{'Edges (raw)':20s}: {n_edges}")
        print()
    
    def extract_and_order_edges(self):
        """Extract edges and order them sequentially"""
        print("=" * 70)
        print("EXTRACTING & ORDERING EDGES")
        print("=" * 70)
        
        topo = TopologyExplorer(self.shape)
        
        # Try to use WireExplorer for proper ordering
        wires = list(topo.wires())
        
        if wires:
            print(f"Found {len(wires)} wire(s)")
            
            # Use the first wire (or combine if multiple)
            wire = wires[0]
            wire_explorer = WireExplorer(wire)
            
            # Get ordered edges from wire
            ordered = []
            for edge in wire_explorer.ordered_edges():
                ordered.append(edge)
            
            self.edges = ordered
            print(f"✓ Extracted {len(ordered)} edges from wire (ordered)")
            
        else:
            # Fallback: manual ordering
            print("No wires found, extracting edges manually...")
            all_edges = list(topo.edges())
            self.edges = self._order_edges_manually(all_edges)
            print(f"✓ Extracted and ordered {len(self.edges)} edges")
        
        # Remove duplicates
        self.edges = self._remove_duplicate_edges(self.edges)
        print(f"✓ After removing duplicates: {len(self.edges)} edges")
        print()
        
        return self.edges
    
    def _remove_duplicate_edges(self, edges):
        """Remove duplicate edges based on start/end points"""
        seen = set()
        unique = []
        
        for edge in edges:
            try:
                curve_adaptor = BRepAdaptor_Curve(edge)
                
                start_pnt = curve_adaptor.Value(curve_adaptor.FirstParameter())
                end_pnt = curve_adaptor.Value(curve_adaptor.LastParameter())
                
                start = Point3D.from_gp_Pnt(start_pnt)
                end = Point3D.from_gp_Pnt(end_pnt)
                
                # Create signature (order-independent)
                sig = tuple(sorted([
                    (round(start.x, 3), round(start.y, 3), round(start.z, 3)),
                    (round(end.x, 3), round(end.y, 3), round(end.z, 3))
                ]))
                
                if sig not in seen:
                    seen.add(sig)
                    unique.append(edge)
            except:
                # Keep edge if we can't analyze it
                unique.append(edge)
        
        return unique
    
    def _order_edges_manually(self, edges):
        """Manually order edges by connectivity"""
        if not edges:
            return []
        
        # Build connectivity map
        edge_data = []
        for edge in edges:
            try:
                curve_adaptor = BRepAdaptor_Curve(edge)
                start_pnt = curve_adaptor.Value(curve_adaptor.FirstParameter())
                end_pnt = curve_adaptor.Value(curve_adaptor.LastParameter())
                
                start = Point3D.from_gp_Pnt(start_pnt)
                end = Point3D.from_gp_Pnt(end_pnt)
                
                edge_data.append((edge, start, end))
            except:
                pass
        
        if not edge_data:
            return edges
        
        # Find starting edge (edge with endpoint that's not connected to many others)
        point_connections = defaultdict(int)
        for _, start, end in edge_data:
            point_connections[start] += 1
            point_connections[end] += 1
        
        # Start with an endpoint (point with only 1 connection)
        start_edge = edge_data[0]
        for edge, start, end in edge_data:
            if point_connections[start] == 1 or point_connections[end] == 1:
                start_edge = (edge, start, end)
                break
        
        # Order edges
        ordered = [start_edge[0]]
        used = {id(start_edge[0])}
        current_point = start_edge[2]  # End point of first edge
        
        while len(ordered) < len(edge_data):
            found = False
            
            for edge, start, end in edge_data:
                if id(edge) in used:
                    continue
                
                # Check if this edge connects to current point
                if start.distance_to(current_point) < 0.1:
                    ordered.append(edge)
                    used.add(id(edge))
                    current_point = end
                    found = True
                    break
                elif end.distance_to(current_point) < 0.1:
                    ordered.append(edge)
                    used.add(id(edge))
                    current_point = start
                    found = True
                    break
            
            if not found:
                # No more connected edges, add remaining
                for edge, _, _ in edge_data:
                    if id(edge) not in used:
                        ordered.append(edge)
                        used.add(id(edge))
                break
        
        return ordered
    
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
            
            # Calculate length
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
        """Consolidate consecutive segments"""
        print("=" * 70)
        print("CONSOLIDATING SEGMENTS")
        print("=" * 70)
        
        if not self.raw_segments:
            return []
        
        consolidated = []
        i = 0
        
        while i < len(self.raw_segments):
            current = self.raw_segments[i]
            
            if current.type == 'STRAIGHT':
                merged = self._merge_collinear_straights(i)
                consolidated.append(merged)
                i += len(merged.sub_edges) if merged.sub_edges else 1
                
            elif current.type == 'BEND':
                merged = self._merge_concentric_arcs(i)
                consolidated.append(merged)
                i += len(merged.sub_edges) if merged.sub_edges else 1
            else:
                consolidated.append(current)
                i += 1
        
        print(f"✓ {len(self.raw_segments)} raw → {len(consolidated)} consolidated segments")
        print()
        
        return consolidated
    
    def _merge_collinear_straights(self, start_idx):
        """Merge collinear straight segments"""
        segments_to_merge = [self.raw_segments[start_idx]]
        
        i = start_idx + 1
        while i < len(self.raw_segments):
            next_seg = self.raw_segments[i]
            
            if next_seg.type != 'STRAIGHT':
                break
            
            prev = segments_to_merge[-1]
            
            # Check collinearity
            if prev.end.distance_to(next_seg.start) < 0.1:
                dir1 = (prev.end.to_array() - prev.start.to_array())
                dir1 = dir1 / (np.linalg.norm(dir1) + 1e-10)
                
                dir2 = (next_seg.end.to_array() - next_seg.start.to_array())
                dir2 = dir2 / (np.linalg.norm(dir2) + 1e-10)
                
                if abs(np.dot(dir1, dir2) - 1.0) < 0.01:
                    segments_to_merge.append(next_seg)
                    i += 1
                else:
                    break
            else:
                break
        
        merged = WireSegment('STRAIGHT', segments_to_merge[0].start, segments_to_merge[-1].end)
        merged.length = sum(s.length for s in segments_to_merge)
        merged.sub_edges = segments_to_merge if len(segments_to_merge) > 1 else []
        
        return merged
    
    def _merge_concentric_arcs(self, start_idx):
        """Merge concentric arc segments"""
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
            
            # Check same center/radius
            center_dist = first_seg.center.distance_to(next_seg.center)
            radius_diff = abs(first_seg.radius - next_seg.radius)
            
            if center_dist < 0.1 and radius_diff < 0.1:
                segments_to_merge.append(next_seg)
                i += 1
            else:
                break
        
        merged = WireSegment('BEND', segments_to_merge[0].start, segments_to_merge[-1].end)
        merged.length = sum(s.length for s in segments_to_merge)
        merged.radius = first_seg.radius
        merged.center = first_seg.center
        merged.sub_edges = segments_to_merge if len(segments_to_merge) > 1 else []
        
        # Calculate total angle
        v1 = segments_to_merge[0].start.to_array() - merged.center.to_array()
        v2 = segments_to_merge[-1].end.to_array() - merged.center.to_array()
        
        v1_norm = v1 / (np.linalg.norm(v1) + 1e-10)
        v2_norm = v2 / (np.linalg.norm(v2) + 1e-10)
        
        cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle_rad = np.arccos(cos_angle)
        merged.angle_deg = np.degrees(angle_rad)
        
        return merged
    
    def analyze_all_segments(self):
        """Analyze all edges"""
        print("=" * 70)
        print("ANALYZING SEGMENTS")
        print("=" * 70)
        
        self.raw_segments = []
        for i, edge in enumerate(self.edges):
            segment = self.analyze_edge(edge, i)
            if segment:
                self.raw_segments.append(segment)
        
        print(f"✓ Analyzed {len(self.raw_segments)} raw segments")
        
        self.segments = self.consolidate_segments()
        
        self.bend_count = sum(1 for s in self.segments if s.type == 'BEND')
        self.straight_count = sum(1 for s in self.segments if s.type == 'STRAIGHT')
        self.total_length = sum(s.length for s in self.segments)
        
        print(f"  Final segments: {len(self.segments)}")
        print(f"  Bends: {self.bend_count}")
        print(f"  Straights: {self.straight_count}")
        print(f"  Total length: {self.total_length:.2f} mm")
        print()
        
        return self.segments
    
    def print_segment_details(self):
        """Print details"""
        print("=" * 70)
        print("SEGMENT DETAILS")
        print("=" * 70)
        print()
        
        for i, segment in enumerate(self.segments, 1):
            print(f"Segment {i}: {segment.type}")
            print(f"  Length: {segment.length:.2f} mm")
            
            if segment.sub_edges:
                print(f"  (Merged from {len(segment.sub_edges)} raw edges)")
            
            if segment.type == 'BEND' and segment.radius:
                print(f"  Radius: {segment.radius:.2f} mm")
                if segment.angle_deg:
                    print(f"  Angle: {segment.angle_deg:.2f}°")
            
            print()
    
    def print_summary(self):
        """Print summary"""
        print("=" * 70)
        print("WIRE ANALYSIS SUMMARY")
        print("=" * 70)
        print()
        
        print(f"Total Length:        {self.total_length:.2f} mm")
        print(f"Number of Bends:     {self.bend_count}")
        print(f"Straight Segments:   {self.straight_count}")
        print()
        
        if self.bend_count > 0:
            print("Bend Details:")
            bend_num = 1
            for segment in self.segments:
                if segment.type == 'BEND':
                    print(f"  Bend {bend_num}: R={segment.radius:.2f}mm, θ={segment.angle_deg:.2f}°, L={segment.length:.2f}mm")
                    bend_num += 1
        
        print()
        print("=" * 70)
        print()


def analyze_step_file(step_file_path: str, output_dir: Optional[str] = None):
    """Main analysis function"""
    print()
    print("#" * 70)
    print(f"# ANALYZING: {Path(step_file_path).name}")
    print("#" * 70)
    print()
    
    analyzer = STEPWireAnalyzer(step_file_path)
    analyzer.load_step_file()
    analyzer.analyze_topology()
    analyzer.extract_and_order_edges()
    analyzer.analyze_all_segments()
    analyzer.print_segment_details()
    analyzer.print_summary()
    
    return analyzer


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        step_file = sys.argv[1]
        analyzer = analyze_step_file(step_file)
    else:
        print("Usage: python step_analyzer_robust.py <step_file>")
