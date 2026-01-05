#!/usr/bin/env python3
"""
STEP Pipe Length Calculator - DIAGNOSTIC VERSION
Shows all surface types found in the file
"""

import sys
import math
from pathlib import Path

from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import (
    GeomAbs_Cylinder, GeomAbs_Torus, GeomAbs_Plane, 
    GeomAbs_Cone, GeomAbs_Sphere, GeomAbs_BSplineSurface,
    GeomAbs_BezierSurface, GeomAbs_SurfaceOfRevolution,
    GeomAbs_SurfaceOfExtrusion
)
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import topods


def get_surface_type_name(surface_type):
    """Convert surface type enum to readable name"""
    type_map = {
        GeomAbs_Cylinder: "CYLINDER",
        GeomAbs_Torus: "TORUS",
        GeomAbs_Plane: "PLANE",
        GeomAbs_Cone: "CONE",
        GeomAbs_Sphere: "SPHERE",
        GeomAbs_BSplineSurface: "B-SPLINE",
        GeomAbs_BezierSurface: "BEZIER",
        GeomAbs_SurfaceOfRevolution: "REVOLUTION",
        GeomAbs_SurfaceOfExtrusion: "EXTRUSION"
    }
    return type_map.get(surface_type, f"UNKNOWN({surface_type})")


def analyze_step_file(step_file):
    """Analyze and show all surfaces in STEP file"""
    print("=" * 60)
    print("STEP FILE SURFACE ANALYZER")
    print("=" * 60)
    print(f"\nLoading: {step_file}")
    
    # Load STEP file
    reader = STEPControl_Reader()
    if reader.ReadFile(str(step_file)) != IFSelect_RetDone:
        raise RuntimeError("Failed to read STEP")
    
    reader.TransferRoots()
    shape = reader.OneShape()
    print("✓ STEP file loaded successfully\n")
    
    # Count surface types
    surface_counts = {}
    surface_details = []
    
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    face_num = 0
    
    while exp.More():
        face_num += 1
        try:
            face = topods.Face(exp.Current())
            surf = BRepAdaptor_Surface(face)
            surf_type = surf.GetType()
            surf_name = get_surface_type_name(surf_type)
            
            # Count
            surface_counts[surf_name] = surface_counts.get(surf_name, 0) + 1
            
            # Get details
            details = {"type": surf_name, "face_num": face_num}
            
            if surf_type == GeomAbs_Cylinder:
                cyl = surf.Cylinder()
                length = abs(surf.LastVParameter() - surf.FirstVParameter())
                details["radius"] = cyl.Radius()
                details["length"] = length
                
            elif surf_type == GeomAbs_Torus:
                tor = surf.Torus()
                angle = abs(surf.LastUParameter() - surf.FirstUParameter())
                details["major_radius"] = tor.MajorRadius()
                details["minor_radius"] = tor.MinorRadius()
                details["angle_deg"] = math.degrees(angle)
                details["arc_length"] = tor.MajorRadius() * angle
            
            surface_details.append(details)
            
        except Exception as e:
            print(f"  ⚠ Error processing face {face_num}: {e}")
        
        exp.Next()
    
    # Print summary
    print("SURFACE TYPE SUMMARY:")
    print("=" * 60)
    for surf_type, count in sorted(surface_counts.items()):
        print(f"  {surf_type:<20} : {count} faces")
    
    print(f"\n  TOTAL FACES: {face_num}")
    print("\n" + "=" * 60)
    
    # Print details
    print("\nDETAILED SURFACE LIST:")
    print("=" * 60)
    
    for detail in surface_details:
        print(f"\nFace {detail['face_num']}: {detail['type']}")
        
        if detail['type'] == "CYLINDER":
            print(f"  Radius: {detail['radius']:.2f} mm")
            print(f"  Length: {detail['length']:.2f} mm")
            
        elif detail['type'] == "TORUS":
            print(f"  Major Radius: {detail['major_radius']:.2f} mm")
            print(f"  Minor Radius: {detail['minor_radius']:.2f} mm")
            print(f"  Angle: {detail['angle_deg']:.1f}°")
            print(f"  Arc Length: {detail['arc_length']:.2f} mm")
    
    print("\n" + "=" * 60)
    print("ANALYSIS COMPLETE")
    print("=" * 60)


def main():
    if len(sys.argv) < 2:
        print("Usage: python diagnose_step.py file.step")
        print("\nThis tool shows all surface types in your STEP file")
        print("Use this to understand why some pipes aren't being detected")
        return
    
    step_file = sys.argv[1]
    analyze_step_file(Path(step_file))


if __name__ == "__main__":
    main()
