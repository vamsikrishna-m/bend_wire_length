"""
STEP File Wire Analyzer - TRUE CATIA Formula
Uses CATIA's exact bend allowance calculation:
V = α×(R+K×T) - 2×(R+T)×tan(min(π/2,α)/2)
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
        self.arc_length = None
        self.direction = None
        self.bend_allowance = None


class STEPWireAnalyzer:
    
    def __init__(self, step_file_path: str, thickness: float = 0.0, k_factor: float = None):
        """
        Args:
            step_file_path: Path to STEP file
            thickness: Material thickness T (0 for wire)
            k_factor: K-factor for neutral axis (auto-calculated if None)
        """
        self.step_file = Path(step_file_path)
        self.shape = None
        self.edges = []
        self.raw_segments = []
        self.segments = []
        self.total_length = 0.0
        self.bend_count = 0
        self.straight_count = 0
        self.thickness = thickness
        self.k_factor = k_factor
        
    def calculate_k_factor(self, radius: float) -> float:
        """
        Calculate K-factor using CATIA's DIN standard formula:
        K = (0.65 + log10(R/T)) / 2
        
        For wire (T=0), use default K=0.33
        """
        if self.k_factor is not None:
            return self.k_factor
        
        if self.thickness <= 0.001:
            # For wire bending (no thickness), use default
            return 0.33
        
        # CATIA DIN formula
        ratio = radius / self.thickness
        k = (0.65 + math.log10(ratio)) / 2.0
        
        # Clamp between 0 and 0.5
        return max(0.0, min(0.5, k))
    
    def calculate_bend_allowance_catia(self, radius: float, angle_deg: float) -> float:
        """
        CATIA's exact bend allowance formula:
        V = α×(R+K×T) - 2×(R+T)×tan(min(π/2,α)/2)
        
        Where:
        - α = bend angle in radians
        - R = inside bend radius
        - T = material thickness
        - K = K-factor (position of neutral fiber)
        
        For wire (T≈0), this simplifies to:
        V = α×R - 2×R×tan(α/2)
        """
        
        # Convert angle to radians
        alpha = math.radians(angle_deg)
        
        # Get K-factor
        K = self.calculate_k_factor(radius)
        
        # CATIA formula
        R = radius
        T = self.thickness
        
        # First term: arc length along neutral axis
        neutral_arc = alpha * (R + K * T)
        
        # Second term: deduction for straight approximation
        # Use min(π/2, α) as per CATIA spec
        alpha_limited = min(math.pi / 2.0, alpha)
        deduction = 2.0 * (R + T) * math.tan(alpha_limited / 2.0)
        
        # Bend allowance
        bend_allowance = neutral_arc - deduction
        
        return bend_allowance
    
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
    
    def analyze_edge(self, edge: TopoDS_Edge, idx: int) -> Optional[WireSegment]:
        try:
            adaptor = BRepAdaptor_Curve(edge)
            curve_type = adaptor.GetType()
            
            start_pnt = adaptor.Value(adaptor.FirstParameter())
            end_pnt = adaptor.Value(adaptor.LastParameter())
            
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
                
                direction = end.to_array() - start.to_array()
                seg.direction = direction / (np.linalg.norm(direction) + 1e-10)
                
                return seg
                
            elif curve_type == GeomAbs_Circle:
                seg = WireSegment('BEND', start, end)
                
                circle = adaptor.Circle()
                radius = circle.Radius()
                center_pnt = circle.Location()
                
                seg.radius = radius
                seg.center = Point3D.from_gp_Pnt(center_pnt)
                seg.arc_length = arc_length
                
                return seg
            
            else:
                seg = WireSegment('STRAIGHT', start, end)
                seg.length = arc_length
                
                direction = end.to_array() - start.to_array()
                seg.direction = direction / (np.linalg.norm(direction) + 1e-10)
                
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
                if prev.direction is not None and next_seg.direction is not None:
                    if abs(np.dot(prev.direction, next_seg.direction) - 1.0) < 0.01:
                        to_merge.append(next_seg)
                        i += 1
                    else:
                        break
                else:
                    break
            else:
                break
        
        merged = WireSegment('STRAIGHT', to_merge[0].start, to_merge[-1].end)
        merged.length = sum(s.length for s in to_merge)
        merged.sub_edges = to_merge if len(to_merge) > 1 else []
        
        if to_merge[0].direction is not None:
            direction = merged.end.to_array() - merged.start.to_array()
            merged.direction = direction / (np.linalg.norm(direction) + 1e-10)
        
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
        
        total_arc_length = sum(s.arc_length for s in to_merge if s.arc_length)
        merged.arc_length = total_arc_length
        
        return merged
    
    def calculate_bend_angles_and_allowances(self):
        """
        Calculate bend angles from adjacent straights and apply CATIA bend allowance
        """
        for i, seg in enumerate(self.segments):
            if seg.type != 'BEND':
                continue
            
            prev_straight = None
            next_straight = None
            
            for j in range(i - 1, -1, -1):
                if self.segments[j].type == 'STRAIGHT':
                    prev_straight = self.segments[j]
                    break
            
            for j in range(i + 1, len(self.segments)):
                if self.segments[j].type == 'STRAIGHT':
                    next_straight = self.segments[j]
                    break
            
            if prev_straight and next_straight:
                if prev_straight.direction is not None and next_straight.direction is not None:
                    # Calculate angle between straights
                    cos_angle = np.dot(prev_straight.direction, next_straight.direction)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle_between = math.degrees(np.arccos(cos_angle))
                    seg.angle_deg = 180.0 - angle_between
                    
                    # Calculate CATIA bend allowance
                    seg.bend_allowance = self.calculate_bend_allowance_catia(
                        seg.radius, seg.angle_deg
                    )
                    
                    # Set length to bend allowance
                    seg.length = seg.bend_allowance
    
    def analyze_all_segments(self):
        print("=" * 70)
        print("ANALYZING (CATIA EXACT Formula)")
        print(f"Thickness: {self.thickness:.3f} mm")
        if self.k_factor:
            print(f"K-factor: {self.k_factor:.3f} (manual)")
        else:
            print(f"K-factor: Auto-calculated per bend")
        print("=" * 70)
        
        self.raw_segments = []
        for i, edge in enumerate(self.edges):
            seg = self.analyze_edge(edge, i)
            if seg:
                self.raw_segments.append(seg)
        
        print(f"✓ {len(self.raw_segments)} raw segments\n")
        
        self.segments = self.consolidate_segments()
        self.calculate_bend_angles_and_allowances()
        
        self.bend_count = sum(1 for s in self.segments if s.type == 'BEND')
        self.straight_count = sum(1 for s in self.segments if s.type == 'STRAIGHT')
        self.total_length = sum(s.length for s in self.segments)
        
        return self.segments
    
    def print_results(self):
        print("=" * 70)
        print("SEGMENT DETAILS (CATIA Formula)")
        print("=" * 70)
        
        for i, seg in enumerate(self.segments, 1):
            print(f"\nSegment {i}: {seg.type}")
            
            if seg.type == 'STRAIGHT':
                print(f"  Length: {seg.length:.2f} mm")
            else:
                print(f"  Bend Allowance: {seg.length:.2f} mm")
            
            if seg.sub_edges:
                print(f"  (Merged from {len(seg.sub_edges)} edges)")
            
            if seg.type == 'BEND':
                if seg.radius:
                    print(f"  Radius: {seg.radius:.2f} mm")
                if seg.angle_deg:
                    print(f"  Angle: {seg.angle_deg:.2f}°")
                if seg.arc_length:
                    print(f"  Arc Length: {seg.arc_length:.2f} mm")
                    
                # Show K-factor used
                K = self.calculate_k_factor(seg.radius)
                print(f"  K-factor: {K:.3f}")
        
        print("\n" + "=" * 70)
        print("WIRE ANALYSIS SUMMARY (CATIA Formula)")
        print("=" * 70)
        print(f"\nTotal Developed Length: {self.total_length:.2f} mm")
        print(f"Number of Bends:        {self.bend_count}")
        print(f"Straight Segments:      {self.straight_count}")
        
        if self.bend_count > 0:
            print(f"\nBend Details:")
            bend_num = 1
            for seg in self.segments:
                if seg.type == 'BEND':
                    print(f"  Bend {bend_num}: R={seg.radius:.2f}mm, θ={seg.angle_deg:.2f}°, BA={seg.bend_allowance:.2f}mm")
                    bend_num += 1
        
        print("\n" + "=" * 70)
        print("Formula Used:")
        print("V = α×(R+K×T) - 2×(R+T)×tan(min(π/2,α)/2)")
        print(f"Where T={self.thickness:.3f}mm")
        print("=" * 70 + "\n")


def analyze_step_file(step_file_path: str, thickness: float = 0.0, k_factor: float = None):
    """
    Analyze STEP file using CATIA's exact bend allowance formula
    
    Args:
        step_file_path: Path to STEP file
        thickness: Material thickness in mm (0 for wire)
        k_factor: K-factor (None for auto-calculation)
    """
    print("\n" + "#" * 70)
    print(f"# {Path(step_file_path).name}")
    print(f"# CATIA EXACT Formula Analysis")
    print("#" * 70 + "\n")
    
    analyzer = STEPWireAnalyzer(step_file_path, thickness=thickness, k_factor=k_factor)
    analyzer.load_step_file()
    analyzer.extract_edges()
    analyzer.analyze_all_segments()
    analyzer.print_results()
    
    return analyzer


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        step_file = sys.argv[1]
        thickness = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
        k_factor = float(sys.argv[3]) if len(sys.argv) > 3 else None
        analyzer = analyze_step_file(step_file, thickness, k_factor)
    else:
        print("Usage: python step_analyzer_catia.py <step_file> [thickness] [k_factor]")
        print("Example: python step_analyzer_catia.py wire.step 0 0.33")
        print("         python step_analyzer_catia.py sheet.step 2.0 0.4")
