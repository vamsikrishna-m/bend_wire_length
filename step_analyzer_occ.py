"""
STEP File Wire Analyzer - TRUE CATIA Compatible
Calculates bend angle from tangent lines like CATIA does
"""

import numpy as np
from pathlib import Path
from typing import List, Optional
import math

# OpenCASCADE imports
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Line, GeomAbs_Circle
from OCC.Core.gp import gp_Pnt, gp_Dir
from OCC.Core.TopoDS import TopoDS_Edge
from OCC.Extend.TopologyUtils import TopologyExplorer, WireExplorer

try:
    from OCC.Core.GCPnts import GCPnts_AbscissaPoint
    HAS_GCPNTS = True
except:
    HAS_GCPNTS = False


class Point3D:
    def __init__(self, x: float, y: float, z: float):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
    
    def distance_to(self, other: 'Point3D') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    def __repr__(self):
        return f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"
    
    @staticmethod
    def from_gp_Pnt(pnt: gp_Pnt) -> 'Point3D':
        return Point3D(pnt.X(), pnt.Y(), pnt.Z())
    
    @staticmethod
    def from_gp_Dir(d: gp_Dir) -> 'Point3D':
        return Point3D(d.X(), d.Y(), d.Z())


class WireSegment:
    def __init__(self, segment_type: str, start: Point3D, end: Point3D):
        self.type = segment_type
        self.start = start
        self.end = end
        self.length = 0.0
        self.radius = None
        self.angle_deg = None
        self.center = None
        self.sub_edges = []
        self.arc_length = None
        self.start_tangent = None
        self.end_tangent = None


class STEPWireAnalyzer:
    
    def __init__(self, step_file_path: str):
        self.step_file = Path(step_file_path)
        self.shape = None
        self.edges = []
        self.raw_segments = []
        self.segments = []
        self.total_length = 0.0
        self.bend_count = 0
        self.straight_count = 0
        
    def load_step_file(self) -> bool:
        print(f"Loading: {self.step_file.name}")
        
        reader = STEPControl_Reader()
        status = reader.ReadFile(str(self.step_file))
        
        if status != IFSelect_RetDone:
            raise Exception(f"Error reading STEP file")
        
        nb_roots = reader.NbRootsForTransfer()
        if nb_roots == 0:
            raise Exception("No transferable roots")
        
        for i in range(1, nb_roots + 1):
            reader.TransferRoot(i)
        
        self.shape = reader.OneShape()
        if self.shape.IsNull():
            raise Exception("Shape is null")
        
        print(f"✓ Loaded\n")
        return True
    
    def extract_edges(self):
        print("=" * 70)
        print("EXTRACTING EDGES")
        print("=" * 70)
        
        topo = TopologyExplorer(self.shape)
        wires = list(topo.wires())
        
        if wires:
            wire = wires[0]
            wire_explorer = WireExplorer(wire)
            self.edges = list(wire_explorer.ordered_edges())
            print(f"✓ {len(self.edges)} edges from wire")
        else:
            self.edges = list(topo.edges())
            print(f"✓ {len(self.edges)} edges")
        
        print()
        return self.edges
    
    def calculate_bend_angle_from_tangents(self, start_tan: Point3D, end_tan: Point3D) -> float:
        """
        Calculate bend angle from tangent vectors (CATIA method)
        The bend angle is the angle BETWEEN the tangent lines, which is
        the SUPPLEMENT of the angle between the tangent vectors
        
        For a 90° bend, tangents are perpendicular, so this returns 90°
        """
        v1 = start_tan.to_array()
        v2 = end_tan.to_array()
        
        # Normalize
        v1_norm = v1 / (np.linalg.norm(v1) + 1e-10)
        v2_norm = v2 / (np.linalg.norm(v2) + 1e-10)
        
        # Angle between tangent vectors
        cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle_between_tangents = math.degrees(np.arccos(cos_angle))
        
        # CATIA measures the bend angle, which is the supplement
        # (180° - angle_between_tangents) for the interior angle
        bend_angle = 180.0 - angle_between_tangents
        
        return bend_angle
    
    def analyze_edge(self, edge: TopoDS_Edge, idx: int) -> Optional[WireSegment]:
        try:
            adaptor = BRepAdaptor_Curve(edge)
            curve_type = adaptor.GetType()
            
            first_param = adaptor.FirstParameter()
            last_param = adaptor.LastParameter()
            
            start_pnt = adaptor.Value(first_param)
            end_pnt = adaptor.Value(last_param)
            
            start = Point3D.from_gp_Pnt(start_pnt)
            end = Point3D.from_gp_Pnt(end_pnt)
            
            # Get arc length
            if HAS_GCPNTS:
                try:
                    arc_length = GCPnts_AbscissaPoint.Length_s(adaptor)
                except:
                    arc_length = start.distance_to(end)
            else:
                arc_length = start.distance_to(end)
            
            if curve_type == GeomAbs_Line:
                seg = WireSegment('STRAIGHT', start, end)
                seg.length = arc_length
                
                # Store tangent for bend calculation
                try:
                    tangent_dir = adaptor.DN(first_param, 1)
                    seg.start_tangent = Point3D.from_gp_Dir(gp_Dir(tangent_dir))
                    seg.end_tangent = seg.start_tangent
                except:
                    pass
                
                return seg
                
            elif curve_type == GeomAbs_Circle:
                seg = WireSegment('BEND', start, end)
                
                circle = adaptor.Circle()
                radius = circle.Radius()
                center_pnt = circle.Location()
                
                seg.radius = radius
                seg.center = Point3D.from_gp_Pnt(center_pnt)
                seg.arc_length = arc_length
                
                # Get tangent vectors at start and end of arc
                try:
                    start_tangent = adaptor.DN(first_param, 1)
                    end_tangent = adaptor.DN(last_param, 1)
                    
                    seg.start_tangent = Point3D.from_gp_Dir(gp_Dir(start_tangent))
                    seg.end_tangent = Point3D.from_gp_Dir(gp_Dir(end_tangent))
                    
                    # Calculate bend angle from tangents (CATIA way)
                    seg.angle_deg = self.calculate_bend_angle_from_tangents(
                        seg.start_tangent, seg.end_tangent
                    )
                except Exception as e:
                    # Fallback to geometric calculation
                    v1 = start.to_array() - seg.center.to_array()
                    v2 = end.to_array() - seg.center.to_array()
                    v1_norm = v1 / (np.linalg.norm(v1) + 1e-10)
                    v2_norm = v2 / (np.linalg.norm(v2) + 1e-10)
                    cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
                    seg.angle_deg = math.degrees(np.arccos(cos_angle))
                
                # Length is arc length
                seg.length = arc_length
                
                return seg
            
            else:
                seg = WireSegment('STRAIGHT', start, end)
                seg.length = arc_length
                return seg
                
        except Exception as e:
            print(f"Warning: edge {idx}: {e}")
            return None
    
    def consolidate_segments(self):
        print("=" * 70)
        print("CONSOLIDATING")
        print("=" * 70)
        
        if not self.raw_segments:
            return []
        
        consolidated = []
        i = 0
        
        while i < len(self.raw_segments):
            current = self.raw_segments[i]
            
            if current.type == 'BEND':
                merged = self._merge_arcs(i)
                consolidated.append(merged)
                i += len(merged.sub_edges) if merged.sub_edges else 1
            else:
                merged = self._merge_straights(i)
                consolidated.append(merged)
                i += len(merged.sub_edges) if merged.sub_edges else 1
        
        print(f"✓ {len(self.raw_segments)} raw → {len(consolidated)} final\n")
        return consolidated
    
    def _merge_straights(self, start_idx):
        to_merge = [self.raw_segments[start_idx]]
        
        i = start_idx + 1
        while i < len(self.raw_segments):
            next_seg = self.raw_segments[i]
            if next_seg.type != 'STRAIGHT':
                break
            
            prev = to_merge[-1]
            if prev.end.distance_to(next_seg.start) < 0.1:
                d1 = (prev.end.to_array() - prev.start.to_array())
                d1 = d1 / (np.linalg.norm(d1) + 1e-10)
                d2 = (next_seg.end.to_array() - next_seg.start.to_array())
                d2 = d2 / (np.linalg.norm(d2) + 1e-10)
                
                if abs(np.dot(d1, d2) - 1.0) < 0.01:
                    to_merge.append(next_seg)
                    i += 1
                else:
                    break
            else:
                break
        
        merged = WireSegment('STRAIGHT', to_merge[0].start, to_merge[-1].end)
        merged.length = sum(s.length for s in to_merge)
        merged.sub_edges = to_merge if len(to_merge) > 1 else []
        
        if to_merge[0].start_tangent:
            merged.start_tangent = to_merge[0].start_tangent
            merged.end_tangent = to_merge[-1].end_tangent
        
        return merged
    
    def _merge_arcs(self, start_idx):
        to_merge = [self.raw_segments[start_idx]]
        
        first = self.raw_segments[start_idx]
        if not first.center or not first.radius:
            return first
        
        i = start_idx + 1
        while i < len(self.raw_segments):
            next_seg = self.raw_segments[i]
            if next_seg.type != 'BEND':
                break
            if not next_seg.center or not next_seg.radius:
                break
            
            if (first.center.distance_to(next_seg.center) < 0.1 and 
                abs(first.radius - next_seg.radius) < 0.1):
                to_merge.append(next_seg)
                i += 1
            else:
                break
        
        merged = WireSegment('BEND', to_merge[0].start, to_merge[-1].end)
        merged.radius = first.radius
        merged.center = first.center
        merged.sub_edges = to_merge if len(to_merge) > 1 else []
        
        # Use first and last tangents for angle calculation
        if to_merge[0].start_tangent and to_merge[-1].end_tangent:
            merged.start_tangent = to_merge[0].start_tangent
            merged.end_tangent = to_merge[-1].end_tangent
            merged.angle_deg = self.calculate_bend_angle_from_tangents(
                merged.start_tangent, merged.end_tangent
            )
        else:
            # Fallback
            v1 = merged.start.to_array() - merged.center.to_array()
            v2 = merged.end.to_array() - merged.center.to_array()
            v1_norm = v1 / (np.linalg.norm(v1) + 1e-10)
            v2_norm = v2 / (np.linalg.norm(v2) + 1e-10)
            cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
            merged.angle_deg = math.degrees(np.arccos(cos_angle))
        
        # Total arc length
        total_arc_length = sum(s.arc_length for s in to_merge if s.arc_length)
        merged.arc_length = total_arc_length
        merged.length = total_arc_length
        
        return merged
    
    def analyze_all_segments(self):
        print("=" * 70)
        print("ANALYZING (TRUE CATIA Method)")
        print("=" * 70)
        
        self.raw_segments = []
        for i, edge in enumerate(self.edges):
            seg = self.analyze_edge(edge, i)
            if seg:
                self.raw_segments.append(seg)
        
        print(f"✓ {len(self.raw_segments)} raw segments\n")
        
        self.segments = self.consolidate_segments()
        
        self.bend_count = sum(1 for s in self.segments if s.type == 'BEND')
        self.straight_count = sum(1 for s in self.segments if s.type == 'STRAIGHT')
        self.total_length = sum(s.length for s in self.segments)
        
        return self.segments
    
    def print_results(self):
        print("=" * 70)
        print("SEGMENT DETAILS (CATIA Method)")
        print("=" * 70)
        
        for i, seg in enumerate(self.segments, 1):
            print(f"\nSegment {i}: {seg.type}")
            print(f"  Length: {seg.length:.2f} mm")
            
            if seg.sub_edges:
                print(f"  (Merged from {len(seg.sub_edges)} edges)")
            
            if seg.type == 'BEND':
                if seg.radius:
                    print(f"  Radius: {seg.radius:.2f} mm")
                if seg.angle_deg:
                    print(f"  Angle: {seg.angle_deg:.2f}°")
                if seg.arc_length:
                    print(f"  Arc Length: {seg.arc_length:.2f} mm")
        
        print("\n" + "=" * 70)
        print("WIRE ANALYSIS SUMMARY (CATIA Method)")
        print("=" * 70)
        print(f"\nTotal Length:        {self.total_length:.2f} mm")
        print(f"Number of Bends:     {self.bend_count}")
        print(f"Straight Segments:   {self.straight_count}")
        
        if self.bend_count > 0:
            print(f"\nBend Details:")
            bend_num = 1
            for seg in self.segments:
                if seg.type == 'BEND':
                    print(f"  Bend {bend_num}: R={seg.radius:.2f}mm, θ={seg.angle_deg:.2f}°, L={seg.length:.2f}mm")
                    bend_num += 1
        
        print("\n" + "=" * 70 + "\n")


def analyze_step_file(step_file_path: str):
    """
    Analyze STEP file with TRUE CATIA-compatible measurements
    """
    print("\n" + "#" * 70)
    print(f"# {Path(step_file_path).name}")
    print(f"# TRUE CATIA-Compatible Analysis")
    print("#" * 70 + "\n")
    
    analyzer = STEPWireAnalyzer(step_file_path)
    analyzer.load_step_file()
    analyzer.extract_edges()
    analyzer.analyze_all_segments()
    analyzer.print_results()
    
    return analyzer


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        step_file = sys.argv[1]
        analyzer = analyze_step_file(step_file)
    else:
        print("Usage: python step_analyzer_catia.py <step_file>")
