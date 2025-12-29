#!/usr/bin/env python3
"""
STEP Pipe Length Calculator - FIXED ADVANCED VERSION
"""

import sys
import math
from pathlib import Path

from OCP.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Torus
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCP.gp import gp_Pnt
from OCP.GC import GC_MakeSegment


class AdvancedPipeLengthCalculator:
    
    def __init__(self, step_file):
        self.step_file = step_file
        self.shape = None
        self.segments = []
        self.total_length = 0.0
        self.centerline_wire = None
        self.debug = True
        
    def load_step(self):
        """Load STEP file using OCP"""
        print(f"Loading STEP file: {self.step_file}")
        
        reader = STEPControl_Reader()
        if reader.ReadFile(str(self.step_file)) != IFSelect_RetDone:
            raise RuntimeError("Failed to read STEP")
        
        reader.TransferRoots()
        self.shape = reader.OneShape()
        print("✓ STEP file loaded successfully")
    
    def extract_centerline(self):
        """Extract pipe centerline from solid geometry"""
        print("\n🔧 Extracting centerline from analytic surfaces...")
        
        exp = TopExp_Explorer(self.shape, TopAbs_FACE)
        
        while exp.More():
            face = TopoDS.Face_s(exp.Current())
            surf = BRepAdaptor_Surface(face)
            
            # ─── CYLINDER (STRAIGHT) ───
            if surf.GetType() == GeomAbs_Cylinder:
                cyl = surf.Cylinder()
                length = abs(surf.LastVParameter() - surf.FirstVParameter())
                
                if length > 1e-6:
                    ax = cyl.Axis()
                    loc = ax.Location()
                    dir_vec = ax.Direction()
                    
                    # Calculate start and end points properly
                    v_start = surf.FirstVParameter()
                    v_end = surf.LastVParameter()
                    
                    start_pt = (
                        loc.X() + dir_vec.X() * v_start,
                        loc.Y() + dir_vec.Y() * v_start,
                        loc.Z() + dir_vec.Z() * v_start
                    )
                    end_pt = (
                        loc.X() + dir_vec.X() * v_end,
                        loc.Y() + dir_vec.Y() * v_end,
                        loc.Z() + dir_vec.Z() * v_end
                    )
                    
                    self.segments.append({
                        "type": "STRAIGHT",
                        "length": length,
                        "radius": cyl.Radius(),
                        "axis_origin": (loc.X(), loc.Y(), loc.Z()),
                        "axis_dir": (dir_vec.X(), dir_vec.Y(), dir_vec.Z()),
                        "start_point": start_pt,
                        "end_point": end_pt
                    })
            
            # ─── TORUS (BEND) ───
            elif surf.GetType() == GeomAbs_Torus:
                tor = surf.Torus()
                angle = abs(surf.LastUParameter() - surf.FirstUParameter())
                length = tor.MajorRadius() * angle
                
                if length > 1e-6:
                    ax = tor.Axis()
                    loc = ax.Location()
                    dir_vec = ax.Direction()
                    
                    self.segments.append({
                        "type": "CIRCULAR_BEND",
                        "length": length,
                        "radius": tor.MajorRadius(),
                        "minor_radius": tor.MinorRadius(),
                        "angle_deg": math.degrees(angle),
                        "center": (loc.X(), loc.Y(), loc.Z()),
                        "axis_dir": (dir_vec.X(), dir_vec.Y(), dir_vec.Z())
                    })
            
            exp.Next()
        
        print(f"✓ Extracted {len(self.segments)} raw segments from solids")
        
        if self.debug:
            print("\n🔍 DEBUG: Raw segments:")
            for i, seg in enumerate(self.segments):
                if seg["type"] == "STRAIGHT":
                    print(f"  {i}: {seg['type']} L={seg['length']:.2f} R={seg['radius']:.2f} "
                          f"dir={seg['axis_dir'][0]:.3f},{seg['axis_dir'][1]:.3f},{seg['axis_dir'][2]:.3f}")
                else:
                    print(f"  {i}: {seg['type']} L={seg['length']:.2f} R={seg['radius']:.2f} ∠={seg['angle_deg']:.1f}°")
    
    def _parallel(self, d1, d2, tol=1e-4):
        """Check if two direction vectors are parallel"""
        dot = d1[0]*d2[0] + d1[1]*d2[1] + d1[2]*d2[2]
        return abs(abs(dot) - 1.0) < tol
    
    def _point_to_axis_distance(self, p, o, d):
        """Calculate distance from point to axis line"""
        vx, vy, vz = p[0]-o[0], p[1]-o[1], p[2]-o[2]
        cx = vy*d[2] - vz*d[1]
        cy = vz*d[0] - vx*d[2]
        cz = vx*d[1] - vy*d[0]
        return math.sqrt(cx*cx + cy*cy + cz*cz)
    
    def merge_duplicates(self, tol=1e-2):
        """Remove duplicate segments created by opposite faces - RELAXED TOLERANCE"""
        print(f"\n🔧 Merging duplicates (tolerance={tol})...")
        merged = []
        
        # RADIUS TOLERANCE: Allow larger difference for inner/outer pipe walls
        radius_tol = 1.0  # 1mm tolerance for radius (handles inner vs outer wall)
        
        for seg in self.segments:
            duplicate = False
            
            for m in merged:
                if seg["type"] != m["type"]:
                    continue
                
                # ─── STRAIGHT SEGMENTS ───
                if seg["type"] == "STRAIGHT":
                    # Check length (relaxed tolerance)
                    if abs(seg["length"] - m["length"]) > tol:
                        continue
                    
                    # Check radius (LARGE tolerance for pipe wall thickness variations)
                    if abs(seg["radius"] - m["radius"]) > radius_tol:
                        continue
                    
                    # Check if axes are the same
                    if "axis_dir" in seg and "axis_dir" in m:
                        if not self._parallel(seg["axis_dir"], m["axis_dir"], tol=1e-3):
                            continue
                        
                        axis_dist = self._point_to_axis_distance(
                            seg["axis_origin"],
                            m["axis_origin"],
                            m["axis_dir"]
                        )
                        
                        if axis_dist < tol:
                            duplicate = True
                            if self.debug:
                                print(f"  Duplicate found: L={seg['length']:.2f} R={seg['radius']:.2f} matches L={m['length']:.2f} R={m['radius']:.2f}")
                
                # ─── CIRCULAR BENDS ───
                elif seg["type"] == "CIRCULAR_BEND":
                    if abs(seg["radius"] - m["radius"]) > radius_tol:
                        continue
                    
                    if abs(seg.get("angle_deg", 0) - m.get("angle_deg", 0)) > 0.5:
                        continue
                    
                    # Check if centers are close (same bend)
                    if "center" in seg and "center" in m:
                        center_dist = math.sqrt(
                            (seg["center"][0] - m["center"][0])**2 +
                            (seg["center"][1] - m["center"][1])**2 +
                            (seg["center"][2] - m["center"][2])**2
                        )
                        if center_dist < tol * 10:  # 10x tolerance for center matching
                            duplicate = True
                            if self.debug:
                                print(f"  Duplicate bend found: R={seg['radius']:.2f} ∠={seg['angle_deg']:.1f}°")
                    else:
                        # Fallback if no center info
                        duplicate = True
                
                if duplicate:
                    break
            
            if not duplicate:
                merged.append(seg)
        
        original_count = len(self.segments)
        self.segments = merged
        print(f"✓ Merged {original_count} → {len(self.segments)} segments")
    
    def build_centerline_wire(self):
        """Build a TopoDS_Wire representing the centerline"""
        print("\n🔧 Building centerline wire...")
        
        wire_builder = BRepBuilderAPI_MakeWire()
        edges_added = 0
        
        for i, seg in enumerate(self.segments):
            try:
                if seg["type"] == "STRAIGHT":
                    # Create line segment
                    p1 = gp_Pnt(*seg["start_point"])
                    p2 = gp_Pnt(*seg["end_point"])
                    
                    edge_maker = BRepBuilderAPI_MakeEdge(p1, p2)
                    if edge_maker.IsDone():
                        edge = edge_maker.Edge()
                        wire_builder.Add(edge)
                        edges_added += 1
                    else:
                        print(f"  ⚠ Could not create edge {i}")
                    
            except Exception as e:
                print(f"  ⚠ Warning: Could not add segment {i} to wire: {e}")
                continue
        
        if wire_builder.IsDone():
            self.centerline_wire = wire_builder.Wire()
            print(f"✓ Centerline wire built successfully ({edges_added} edges)")
        else:
            print(f"⚠ Warning: Wire building incomplete ({edges_added} edges added)")
            self.centerline_wire = None
    
    def export_centerline_step(self, output_file):
        """Export centerline as STEP file"""
        if self.centerline_wire is None:
            print("⚠ No centerline wire to export")
            return False
        
        print(f"\n💾 Exporting centerline to STEP: {output_file}")
        
        writer = STEPControl_Writer()
        writer.Transfer(self.centerline_wire, STEPControl_AsIs)
        status = writer.Write(str(output_file))
        
        if status == IFSelect_RetDone:
            print(f"✓ Centerline exported to: {output_file}")
            return True
        else:
            print(f"✗ Failed to export centerline")
            return False
    
    def calculate(self, export_step=False):
        """Main calculation pipeline"""
        print("=" * 60)
        print("STEP PIPE LENGTH CALCULATOR - ADVANCED")
        print("=" * 60)
        
        self.load_step()
        self.extract_centerline()
        self.merge_duplicates()
        
        self.total_length = sum(s["length"] for s in self.segments)
        
        self.display_results()
        
        # Build centerline wire
        self.build_centerline_wire()
        
        # Export if requested
        if export_step and self.centerline_wire:
            output_step = Path(self.step_file).stem + "_centerline.step"
            self.export_centerline_step(output_step)
        
        return self.total_length
    
    def display_results(self):
        """Print results"""
        print("\nRESULTS")
        print("=" * 60)
        print(f"Total Segments: {len(self.segments)}")
        
        for i, s in enumerate(self.segments, 1):
            seg_type = s["type"]
            length = s["length"]
            radius = s.get("radius", 0)
            
            if seg_type == "CIRCULAR_BEND":
                angle = s.get("angle_deg", 0)
                print(f"{i}. {seg_type:<20} {length:>10.4f} mm  (R={radius:.2f}, ∠={angle:.1f}°)")
            else:
                print(f"{i}. {seg_type:<20} {length:>10.4f} mm  (R={radius:.2f})")
        
        print("\n" + "=" * 60)
        print(f"TOTAL CENTERLINE LENGTH: {self.total_length:.4f} mm")
        print(f"                        {self.total_length/1000:.4f} meters")
        print("=" * 60)


def main():
    if len(sys.argv) < 2:
        print("Usage: python advanced_fixed.py file.step [--export-step]")
        return
    
    step_file = sys.argv[1]
    export_step = "--export-step" in sys.argv
    
    calc = AdvancedPipeLengthCalculator(Path(step_file))
    calc.calculate(export_step=export_step)


if __name__ == "__main__":
    main()
