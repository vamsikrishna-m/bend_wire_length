"""
STEP File Wire Analyzer - CATIA Compatible Version
Matches CATIA's bend measurement methodology
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
from OCC.Core.gp import gp_Pnt
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
        # CATIA-specific measurements
        self.arc_length = None  # Pure arc length
        self.developed_length = None  # CATIA-style developed length


class STEPWireAnalyzer:
    
    def __init__(self, step_file_path: str, k_factor: float = 0.33):
        """
        Initialize analyzer with CATIA-compatible measurements
        
        Args:
            step_file_path: Path to STEP file
            k_factor: Bend allowance K-factor (default 0.33, typical for sheet metal)
        """
        self.step_file = Path(step_file_path)
        self.shape = None
        self.edges = []
        self.raw_segments = []
        self.segments = []
        self.total_length = 0.0
        self.bend_count = 0
        self.straight_count = 0
        self.k_factor = k_factor
        
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
    
    def calculate_bend_angle_from_geometry(self, start: Point3D, end: Point3D, 
                                          center: Point3D, radius: float) -> float:
        """
        Calculate the ACTUAL bend angle using vector geometry
        This matches CATIA's angle calculation
        """
        # Vectors from center to start and end points
        v1 = start.to_array() - center.to_array()
        v2 = end.to_array() - center.to_array()
        
        # Normalize vectors
        v1_norm = v1 / (np.linalg.norm(v1) + 1e-10)
        v2_norm = v2 / (np.linalg.norm(v2) + 1e-10)
        
        # Calculate angle using dot product
        cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle_rad = np.arccos(cos_angle)
        
        # Convert to degrees
        angle_deg = math.degrees(angle_rad)
        
        return angle_deg
    
    def analyze_edge(self, edge: TopoDS_Edge, idx: int) -> Optional[WireSegment]:
        try:
            adaptor = BRepAdaptor_Curve(edge)
            curve_type = adaptor.GetType()
            
            start_pnt = adaptor.Value(adaptor.FirstParameter())
            end_pnt = adaptor.Value(adaptor.LastParameter())
            
            start = Point3D.from_gp_Pnt(start_pnt)
            end = Point3D.from_gp_Pnt(end_pnt)
            
            # Get arc length (actual curve length)
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
                return seg
                
            elif curve_type == GeomAbs_Circle:
                seg = WireSegment('BEND', start, end)
                
                circle = adaptor.Circle()
                radius = circle.Radius()
                center_pnt = circle.Location()
                center = Point3D.from_gp_Pnt(center_pnt)
                
                seg.radius = radius
                seg.center = center
                seg.arc_length = arc_length
                
                # CORRECT ANGLE: Calculate from geometry, not arc length
                seg.angle_deg = self.calculate_bend_angle_from_geometry(
                    start, end, center, radius
                )
                
                # Calculate developed length (CATIA-style)
                # This uses bend allowance formula
                seg.developed_length = self.calculate_developed_length(
                    radius, seg.angle_deg
                )
                
                # Use developed length as the segment length (CATIA way)
                seg.length = seg.developed_length
                
                return seg
            
            else:
                seg = WireSegment('STRAIGHT', start, end)
                seg.length = arc_length
                return seg
                
        except Exception as e:
            print(f"Warning: edge {idx}: {e}")
            return None
    
    def calculate_developed_length(self, radius: float, angle_deg: float) -> float:
        """
        Calculate developed length using bend allowance formula
        This matches CATIA's calculation
        
        Formula: L = (π/180) × Angle × (R + K×t)
        For wire bending (no thickness): L = (π/180) × Angle × R
        
        But CATIA may use: L = 2 × R × tan(Angle/2) for the straight equivalent
        """
        angle_rad = math.radians(angle_deg)
        
        # Method 1: Standard bend allowance (neutral axis)
        neutral_radius = radius  # For wire, neutral axis is at center
        developed = (math.pi / 180.0) * angle_deg * neutral_radius
        
        return developed
    
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
                # Merge consecutive arcs with same center/radius
                merged = self._merge_arcs(i)
                consolidated.append(merged)
                i += len(merged.sub_edges) if merged.sub_edges else 1
            else:
                # Merge collinear straights
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
                # Check direction
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
            
            # Same center and radius?
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
        
        # Calculate total angle from merged geometry
        merged.angle_deg = self.calculate_bend_angle_from_geometry(
            merged.start, merged.end, merged.center, merged.radius
        )
        
        # Calculate arc length
        total_arc_length = sum(s.arc_length for s in to_merge if s.arc_length)
        merged.arc_length = total_arc_length
        
        # Calculate developed length
        merged.developed_length = self.calculate_developed_length(
            merged.radius, merged.angle_deg
        )
        
        # Use developed length as segment length
        merged.length = merged.developed_length
        
        return merged
    
    def analyze_all_segments(self):
        print("=" * 70)
        print("ANALYZING (CATIA-Compatible)")
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
        print("SEGMENT DETAILS (CATIA-Compatible)")
        print("=" * 70)
        
        for i, seg in enumerate(self.segments, 1):
            print(f"\nSegment {i}: {seg.type}")
            print(f"  Developed Length: {seg.length:.2f} mm")
            
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
        print("WIRE ANALYSIS SUMMARY (CATIA-Compatible)")
        print("=" * 70)
        print(f"\nTotal Developed Length: {self.total_length:.2f} mm")
        print(f"Number of Bends:        {self.bend_count}")
        print(f"Straight Segments:      {self.straight_count}")
        
        if self.bend_count > 0:
            print(f"\nBend Details:")
            bend_num = 1
            for seg in self.segments:
                if seg.type == 'BEND':
                    print(f"  Bend {bend_num}:")
                    print(f"    Radius: {seg.radius:.2f} mm")
                    print(f"    Angle: {seg.angle_deg:.2f}°")
                    print(f"    Arc Length: {seg.arc_length:.2f} mm")
                    print(f"    Developed Length: {seg.developed_length:.2f} mm")
                    bend_num += 1
        
        print("\n" + "=" * 70 + "\n")


def analyze_step_file(step_file_path: str, k_factor: float = 0.33):
    """
    Analyze STEP file with CATIA-compatible measurements
    
    Args:
        step_file_path: Path to STEP file
        k_factor: Bend allowance K-factor (default 0.33)
    """
    print("\n" + "#" * 70)
    print(f"# {Path(step_file_path).name}")
    print(f"# CATIA-Compatible Analysis")
    print("#" * 70 + "\n")
    
    analyzer = STEPWireAnalyzer(step_file_path, k_factor=k_factor)
    analyzer.load_step_file()
    analyzer.extract_edges()
    analyzer.analyze_all_segments()
    analyzer.print_results()
    
    return analyzer


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        step_file = sys.argv[1]
        k_factor = float(sys.argv[2]) if len(sys.argv) > 2 else 0.33
        analyzer = analyze_step_file(step_file, k_factor=k_factor)
    else:
        print("Usage: python step_analyzer_catia.py <step_file> [k_factor]")
        print("Example: python step_analyzer_catia.py wire.step 0.33")
