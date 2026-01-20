import sys
import math
from pathlib import Path

from OCP.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Torus
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCP.gp import gp_Pnt
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.TopExp import TopExp


class UniversalPipeLengthCalculator:
    """
    Keeps EXACT same merging logic as original advanced_fixed.py
    ONLY adds: separation of individual pipes in assemblies
    """
    
    def __init__(self, step_file):
        self.step_file = step_file
        self.shape = None
        self.pipes = []
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
    
    def separate_solids(self):
        """Separate individual solid bodies (pipes) in the assembly"""
        print("\n🔧 Separating individual pipes...")
        
        solids_map = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(self.shape, TopAbs_SOLID, solids_map)
        
        num_solids = solids_map.Size()
        print(f"✓ Found {num_solids} solid bodies")
        
        return [solids_map.FindKey(i+1) for i in range(num_solids)]
    
    def extract_pipe_segments(self, solid, pipe_id):
        """Extract centerline segments from a single pipe solid"""
        segments = []
        
        exp = TopExp_Explorer(solid, TopAbs_FACE)
        
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
                    
                    segments.append({
                        "type": "STRAIGHT",
                        "length": length,
                        "radius": cyl.Radius(),
                        "axis_origin": (loc.X(), loc.Y(), loc.Z()),
                        "axis_dir": (dir_vec.X(), dir_vec.Y(), dir_vec.Z()),
                        "start_point": start_pt,
                        "end_point": end_pt,
                        "pipe_id": pipe_id
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
                    
                    segments.append({
                        "type": "CIRCULAR_BEND",
                        "length": length,
                        "radius": tor.MajorRadius(),
                        "minor_radius": tor.MinorRadius(),
                        "angle_deg": math.degrees(angle),
                        "center": (loc.X(), loc.Y(), loc.Z()),
                        "axis_dir": (dir_vec.X(), dir_vec.Y(), dir_vec.Z()),
                        "pipe_id": pipe_id
                    })
            
            exp.Next()
        
        return segments
    
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
    
    def merge_duplicates(self, segments, tol=1e-2):
        """
        EXACT SAME LOGIC AS ORIGINAL advanced_fixed.py
        Remove duplicate segments created by opposite faces
        """
        merged = []
        
        # RADIUS TOLERANCE: Allow larger difference for inner/outer pipe walls
        radius_tol = 1.0  # 1mm tolerance for radius (handles inner vs outer wall)
        
        for seg in segments:
            duplicate = False
            
            for m in merged:
                if seg["type"] != m["type"]:
                    continue
                
                # ─── STRAIGHT SEGMENTS ───
                if seg["type"] == "STRAIGHT":
                    # Check length (relaxed tolerance)
                    if abs(seg["length"] - m["length"]) > tol:
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
                        
                        # If same length AND coaxial -> it's a duplicate (inner/outer wall)
                        # DO NOT check radius - inner and outer walls have different radii!
                        if axis_dist < tol:
                            duplicate = True
                            if self.debug:
                                print(f"  Duplicate found: L={seg['length']:.2f} R={seg['radius']:.2f} matches L={m['length']:.2f} R={m['radius']:.2f}")
                
                # ─── CIRCULAR BENDS ───
                elif seg["type"] == "CIRCULAR_BEND":
                    # Check angle (must match closely)
                    if abs(seg.get("angle_deg", 0) - m.get("angle_deg", 0)) > 0.5:
                        continue
                    
                    # Check if centers are close (same bend)
                    # DO NOT check radius - inner and outer bends have different radii!
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
        
        return merged
    
    def process_assembly(self):
        """Process entire assembly - separate pipes and calculate each"""
        print("\n" + "="*60)
        print("PROCESSING PIPE ASSEMBLY")
        print("="*60)
        
        # Separate individual solids
        solids = self.separate_solids()
        
        # Process each solid as a separate pipe
        for i, solid in enumerate(solids):
            pipe_id = i + 1
            print(f"\n📏 Processing Pipe #{pipe_id}...")
            
            # Extract segments from this pipe
            raw_segments = self.extract_pipe_segments(solid, pipe_id)
            print(f"  Found {len(raw_segments)} raw segments")
            
            # Merge duplicates using ORIGINAL logic
            merged_segments = self.merge_duplicates(raw_segments)
            print(f"  After merge: {len(merged_segments)} unique segments")
            
            # Calculate total length for this pipe
            total_length = sum(s["length"] for s in merged_segments)
            
            pipe_info = {
                "pipe_id": pipe_id,
                "segments": merged_segments,
                "total_length": total_length
            }
            
            self.pipes.append(pipe_info)
            
            print(f"  ✓ Pipe #{pipe_id} total length: {total_length:.2f} mm ({total_length/1000:.4f} m)")
    
    def display_results(self):
        """Display comprehensive results"""
        print("\n" + "="*60)
        print("RESULTS SUMMARY")
        print("="*60)
        
        grand_total = 0.0
        
        for pipe in self.pipes:
            pipe_id = pipe["pipe_id"]
            segments = pipe["segments"]
            total = pipe["total_length"]
            
            print(f"\n📌 PIPE #{pipe_id}")
            print(f"   Segments: {len(segments)}")
            
            for i, seg in enumerate(segments, 1):
                seg_type = seg["type"]
                length = seg["length"]
                radius = seg.get("radius", 0)
                
                if seg_type == "CIRCULAR_BEND":
                    angle = seg.get("angle_deg", 0)
                    print(f"   {i}. {seg_type:<18} {length:>10.4f} mm  (R={radius:.2f}, ∠={angle:.1f}°)")
                else:
                    print(f"   {i}. {seg_type:<18} {length:>10.4f} mm  (R={radius:.2f})")
            
            print(f"\n   PIPE #{pipe_id} TOTAL: {total:.4f} mm ({total/1000:.4f} m)")
            print("   " + "-"*55)
            
            grand_total += total
        
        print("\n" + "="*60)
        print(f"TOTAL PIPES: {len(self.pipes)}")
        print(f"GRAND TOTAL LENGTH: {grand_total:.4f} mm")
        print(f"                    {grand_total/1000:.4f} meters")
        print("="*60)
        
        return grand_total
    
    def build_centerline_wire(self, segments):
        """Build a TopoDS_Wire representing the centerline"""
        wire_builder = BRepBuilderAPI_MakeWire()
        
        for seg in segments:
            if seg["type"] == "STRAIGHT":
                try:
                    p1 = gp_Pnt(*seg["start_point"])
                    p2 = gp_Pnt(*seg["end_point"])
                    
                    edge_maker = BRepBuilderAPI_MakeEdge(p1, p2)
                    if edge_maker.IsDone():
                        edge = edge_maker.Edge()
                        wire_builder.Add(edge)
                except:
                    pass
        
        if wire_builder.IsDone():
            return wire_builder.Wire()
        return None
    
    def export_centerlines_step(self, output_file):
        """Export all pipe centerlines to a single STEP file"""
        print(f"\n💾 Exporting centerlines to STEP: {output_file}")
        
        writer = STEPControl_Writer()
        total_exported = 0
        
        for pipe in self.pipes:
            wire = self.build_centerline_wire(pipe["segments"])
            if wire:
                writer.Transfer(wire, STEPControl_AsIs)
                total_exported += 1
        
        status = writer.Write(str(output_file))
        
        if status == IFSelect_RetDone:
            print(f"✓ Exported {total_exported} pipe centerlines")
            return True
        else:
            print(f"✗ Failed to export centerlines")
            return False
    
    def calculate(self, export_step=False):
        """Main calculation pipeline"""
        print("="*60)
        print("UNIVERSAL PIPE LENGTH CALCULATOR")
        print("Same merging logic as advanced_fixed.py")
        print("="*60)
        
        self.load_step()
        self.process_assembly()
        grand_total = self.display_results()
        
        if export_step:
            output_step = Path(self.step_file).stem + "_centerlines.step"
            self.export_centerlines_step(output_step)
        
        return grand_total


def main():
    if len(sys.argv) < 2:
        print("Usage: python universal_pipe_calculator.py file.step [--export-step]")
        return
    
    step_file = sys.argv[1]
    export_step = "--export-step" in sys.argv
    
    calc = UniversalPipeLengthCalculator(Path(step_file))
    calc.calculate(export_step=export_step)


if __name__ == "__main__":
    main()
