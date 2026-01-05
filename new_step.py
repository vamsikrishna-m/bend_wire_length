#!/usr/bin/env python3
"""
STEP Pipe Length Calculator - BSPLINE/NURBS COMPATIBLE VERSION
Handles both analytic surfaces AND approximated BSpline/NURBS geometry
"""

import sys
import math
import numpy as np
from pathlib import Path
from collections import defaultdict

from OCP.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCP.GeomAbs import (GeomAbs_Cylinder, GeomAbs_Torus, GeomAbs_Plane, 
                          GeomAbs_BSplineSurface, GeomAbs_BezierSurface)
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir, gp_Circ, gp_Vec
from OCP.GC import GC_MakeSegment, GC_MakeArcOfCircle
from OCP.BRep import BRep_Tool
from OCP.GeomLProp import GeomLProp_SLProps


class UniversalPipeLengthCalculator:
    
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
    
    def sample_surface_points(self, surf, n_u=20, n_v=20):
        """Sample points on a parametric surface"""
        u_min = surf.FirstUParameter()
        u_max = surf.LastUParameter()
        v_min = surf.FirstVParameter()
        v_max = surf.LastVParameter()
        
        points = []
        normals = []
        
        for i in range(n_u):
            u = u_min + (u_max - u_min) * i / (n_u - 1)
            for j in range(n_v):
                v = v_min + (v_max - v_min) * j / (n_v - 1)
                
                try:
                    pnt = surf.Value(u, v)
                    points.append([pnt.X(), pnt.Y(), pnt.Z()])
                    
                    # Get normal
                    props = GeomLProp_SLProps(surf, u, v, 1, 1e-6)
                    if props.IsNormalDefined():
                        normal = props.Normal()
                        normals.append([normal.X(), normal.Y(), normal.Z()])
                    else:
                        normals.append([0, 0, 0])
                except:
                    continue
        
        return np.array(points), np.array(normals)
    
    def fit_cylinder_to_points(self, points, normals):
        """Fit a cylinder to a point cloud
        Returns: (axis_origin, axis_direction, radius, length) or None
        """
        if len(points) < 10:
            return None
        
        try:
            # Step 1: Find approximate axis using PCA on normals
            # For a cylinder, normals should be perpendicular to axis
            if len(normals) > 0 and np.any(normals):
                # Average normal directions
                norm_mean = np.mean(normals, axis=0)
                
                # Use cross products to find axis direction
                # Axis is perpendicular to normals
                u, s, vt = np.linalg.svd(normals - norm_mean)
                axis_dir = vt[-1]  # Least variance = axis direction
            else:
                # Fallback: use PCA on points
                centroid = np.mean(points, axis=0)
                u, s, vt = np.linalg.svd(points - centroid)
                axis_dir = vt[0]  # Most variance = axis direction
            
            axis_dir = axis_dir / np.linalg.norm(axis_dir)
            
            # Step 2: Project points onto plane perpendicular to axis
            centroid = np.mean(points, axis=0)
            
            # Calculate distances from axis
            radii = []
            projected_points = []
            
            for pt in points:
                # Vector from centroid to point
                v = pt - centroid
                # Project onto axis
                proj_len = np.dot(v, axis_dir)
                # Point on axis closest to pt
                axis_point = centroid + proj_len * axis_dir
                # Radial distance
                radius = np.linalg.norm(pt - axis_point)
                radii.append(radius)
                projected_points.append(proj_len)
            
            # Step 3: Calculate cylinder parameters
            radius = np.median(radii)
            length = max(projected_points) - min(projected_points)
            
            # Quality check: radii should be consistent
            radius_std = np.std(radii)
            if radius_std / radius > 0.15:  # More than 15% variation
                return None
            
            # Calculate actual start and end points
            proj_min = min(projected_points)
            proj_max = max(projected_points)
            
            start_point = centroid + proj_min * axis_dir
            end_point = centroid + proj_max * axis_dir
            
            return {
                "type": "STRAIGHT",
                "length": length,
                "radius": radius,
                "axis_origin": tuple(centroid),
                "axis_dir": tuple(axis_dir),
                "start_point": tuple(start_point),
                "end_point": tuple(end_point),
                "quality": 1.0 - radius_std / radius
            }
            
        except Exception as e:
            if self.debug:
                print(f"  Warning: Cylinder fitting failed: {e}")
            return None
    
    def fit_torus_to_points(self, points, normals):
        """Fit a torus (bend) to a point cloud
        Returns: segment dict or None
        """
        if len(points) < 20:
            return None
        
        try:
            # Step 1: Find the center plane using PCA
            centroid = np.mean(points, axis=0)
            u, s, vt = np.linalg.svd(points - centroid)
            
            # The bend plane normal is the direction with least variance
            plane_normal = vt[-1]
            plane_normal = plane_normal / np.linalg.norm(plane_normal)
            
            # Step 2: Project points onto the bend plane
            projected = []
            for pt in points:
                v = pt - centroid
                proj_len = np.dot(v, plane_normal)
                proj_pt = pt - proj_len * plane_normal
                projected.append(proj_pt)
            
            projected = np.array(projected)
            
            # Step 3: Fit a circle to the projected points
            # This gives us the major radius (centerline)
            proj_centroid = np.mean(projected, axis=0)
            
            distances = []
            for pt in projected:
                dist = np.linalg.norm(pt - proj_centroid)
                distances.append(dist)
            
            major_radius = np.median(distances)
            
            # Step 4: Estimate the arc angle
            # Use the angular span of points around the circle
            angles = []
            for pt in projected:
                v = pt - proj_centroid
                # Project onto plane coordinate system
                angle = np.arctan2(v[1], v[0])
                angles.append(angle)
            
            angle_span = max(angles) - min(angles)
            if angle_span < 0:
                angle_span += 2 * np.pi
            
            # Step 5: Calculate arc length
            arc_length = major_radius * angle_span
            
            # Quality check
            dist_std = np.std(distances)
            if dist_std / major_radius > 0.2:  # More than 20% variation
                return None
            
            return {
                "type": "CIRCULAR_BEND",
                "length": arc_length,
                "radius": major_radius,
                "minor_radius": 0,  # Unknown for approximation
                "angle_deg": math.degrees(angle_span),
                "angle_rad": angle_span,
                "center": tuple(proj_centroid),
                "axis_dir": tuple(plane_normal),
                "u_start": 0,
                "u_end": angle_span,
                "quality": 1.0 - dist_std / major_radius
            }
            
        except Exception as e:
            if self.debug:
                print(f"  Warning: Torus fitting failed: {e}")
            return None
    
    def classify_and_fit_surface(self, surf):
        """Classify surface type and fit appropriate geometry"""
        # Sample the surface
        points, normals = self.sample_surface_points(surf, n_u=15, n_v=15)
        
        if len(points) < 10:
            return None
        
        # Try cylinder fit first
        cyl_fit = self.fit_cylinder_to_points(points, normals)
        if cyl_fit and cyl_fit.get("quality", 0) > 0.7:
            return cyl_fit
        
        # Try torus fit
        tor_fit = self.fit_torus_to_points(points, normals)
        if tor_fit and tor_fit.get("quality", 0) > 0.6:
            return tor_fit
        
        return None
    
    def extract_centerline(self):
        """Extract pipe centerline from solid geometry"""
        print("\n🔧 Extracting centerline from geometry...")
        
        exp = TopExp_Explorer(self.shape, TopAbs_FACE)
        face_count = 0
        
        while exp.More():
            face = TopoDS.Face_s(exp.Current())
            surf = BRepAdaptor_Surface(face)
            surf_type = surf.GetType()
            face_count += 1
            
            segment = None
            
            # ─── ANALYTIC CYLINDER (STRAIGHT) ───
            if surf_type == GeomAbs_Cylinder:
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
                    
                    segment = {
                        "type": "STRAIGHT",
                        "length": length,
                        "radius": cyl.Radius(),
                        "axis_origin": (loc.X(), loc.Y(), loc.Z()),
                        "axis_dir": (dir_vec.X(), dir_vec.Y(), dir_vec.Z()),
                        "start_point": start_pt,
                        "end_point": end_pt,
                        "source": "ANALYTIC_CYLINDER"
                    }
            
            # ─── ANALYTIC TORUS (BEND) ───
            elif surf_type == GeomAbs_Torus:
                tor = surf.Torus()
                angle = abs(surf.LastUParameter() - surf.FirstUParameter())
                length = tor.MajorRadius() * angle
                
                if length > 1e-6:
                    ax = tor.Axis()
                    loc = ax.Location()
                    dir_vec = ax.Direction()
                    
                    u_start = surf.FirstUParameter()
                    u_end = surf.LastUParameter()
                    
                    segment = {
                        "type": "CIRCULAR_BEND",
                        "length": length,
                        "radius": tor.MajorRadius(),
                        "minor_radius": tor.MinorRadius(),
                        "angle_deg": math.degrees(angle),
                        "angle_rad": angle,
                        "center": (loc.X(), loc.Y(), loc.Z()),
                        "axis_dir": (dir_vec.X(), dir_vec.Y(), dir_vec.Z()),
                        "u_start": u_start,
                        "u_end": u_end,
                        "source": "ANALYTIC_TORUS"
                    }
            
            # ─── BSPLINE/NURBS SURFACE (APPROXIMATE) ───
            elif surf_type in [GeomAbs_BSplineSurface, GeomAbs_BezierSurface]:
                segment = self.classify_and_fit_surface(surf)
                if segment:
                    segment["source"] = "APPROXIMATED"
            
            if segment:
                self.segments.append(segment)
            
            exp.Next()
        
        print(f"✓ Extracted {len(self.segments)} raw segments from {face_count} faces")
        
        if self.debug and len(self.segments) > 0:
            print("\n🔍 DEBUG: Raw segments:")
            for i, seg in enumerate(self.segments):
                source = seg.get("source", "UNKNOWN")
                if seg["type"] == "STRAIGHT":
                    print(f"  {i}: {seg['type']} L={seg['length']:.2f} R={seg['radius']:.2f} [{source}]")
                else:
                    print(f"  {i}: {seg['type']} L={seg['length']:.2f} R={seg['radius']:.2f} ∠={seg['angle_deg']:.1f}° [{source}]")
    
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
    
    def merge_duplicates(self, tol=5.0):
        """Remove duplicate segments - MORE AGGRESSIVE for approximated geometry"""
        if len(self.segments) == 0:
            return
            
        print(f"\n🔧 Merging duplicates (tolerance={tol}mm)...")
        merged = []
        
        # LARGE tolerance for approximated geometry
        radius_tol = 5.0  # 5mm tolerance for radius
        
        for seg in self.segments:
            duplicate = False
            
            for m in merged:
                if seg["type"] != m["type"]:
                    continue
                
                # ─── STRAIGHT SEGMENTS ───
                if seg["type"] == "STRAIGHT":
                    if abs(seg["length"] - m["length"]) > tol:
                        continue
                    
                    if abs(seg["radius"] - m["radius"]) > radius_tol:
                        continue
                    
                    if "axis_dir" in seg and "axis_dir" in m:
                        if not self._parallel(seg["axis_dir"], m["axis_dir"], tol=0.1):
                            continue
                        
                        axis_dist = self._point_to_axis_distance(
                            seg["axis_origin"],
                            m["axis_origin"],
                            m["axis_dir"]
                        )
                        
                        if axis_dist < tol * 2:
                            duplicate = True
                
                # ─── CIRCULAR BENDS ───
                elif seg["type"] == "CIRCULAR_BEND":
                    if abs(seg["radius"] - m["radius"]) > radius_tol:
                        continue
                    
                    if abs(seg.get("angle_deg", 0) - m.get("angle_deg", 0)) > 5.0:
                        continue
                    
                    if "center" in seg and "center" in m:
                        center_dist = math.sqrt(
                            (seg["center"][0] - m["center"][0])**2 +
                            (seg["center"][1] - m["center"][1])**2 +
                            (seg["center"][2] - m["center"][2])**2
                        )
                        if center_dist < tol * 3:
                            duplicate = True
                
                if duplicate:
                    break
            
            if not duplicate:
                merged.append(seg)
        
        print(f"✓ Merged {len(self.segments)} → {len(merged)} segments")
        self.segments = merged
    
    def build_centerline_wire(self):
        """Build a TopoDS_Wire representing the centerline"""
        if len(self.segments) == 0:
            return
            
        print("\n🔧 Building centerline wire...")
        
        wire_builder = BRepBuilderAPI_MakeWire()
        edges_added = 0
        
        for i, seg in enumerate(self.segments):
            try:
                if seg["type"] == "STRAIGHT":
                    p1 = gp_Pnt(*seg["start_point"])
                    p2 = gp_Pnt(*seg["end_point"])
                    
                    edge_maker = BRepBuilderAPI_MakeEdge(p1, p2)
                    if edge_maker.IsDone():
                        wire_builder.Add(edge_maker.Edge())
                        edges_added += 1
                
                elif seg["type"] == "CIRCULAR_BEND":
                    center = gp_Pnt(*seg["center"])
                    axis_dir = gp_Dir(*seg["axis_dir"])
                    
                    ax_dir = seg["axis_dir"]
                    if abs(ax_dir[2]) < 0.9:
                        ref_x = gp_Dir(0, 0, 1)
                    else:
                        ref_x = gp_Dir(1, 0, 0)
                    
                    ax2 = gp_Ax2(center, axis_dir, ref_x)
                    circ = gp_Circ(ax2, seg["radius"])
                    
                    u_start = seg["u_start"]
                    u_end = seg["u_end"]
                    
                    arc_maker = GC_MakeArcOfCircle(circ, u_start, u_end, True)
                    
                    if arc_maker.IsDone():
                        edge_maker = BRepBuilderAPI_MakeEdge(arc_maker.Value())
                        if edge_maker.IsDone():
                            wire_builder.Add(edge_maker.Edge())
                            edges_added += 1
                    
            except Exception as e:
                if self.debug:
                    print(f"  ⚠ Warning: Could not add segment {i}: {e}")
                continue
        
        if wire_builder.IsDone():
            self.centerline_wire = wire_builder.Wire()
            print(f"✓ Centerline wire built ({edges_added} edges)")
        else:
            print(f"⚠ Wire incomplete ({edges_added} edges)")
    
    def export_centerline_step(self, output_file):
        """Export centerline as STEP file"""
        if self.centerline_wire is None:
            return False
        
        print(f"\n💾 Exporting centerline: {output_file}")
        
        writer = STEPControl_Writer()
        writer.Transfer(self.centerline_wire, STEPControl_AsIs)
        status = writer.Write(str(output_file))
        
        return status == IFSelect_RetDone
    
    def calculate(self, export_step=False):
        """Main calculation pipeline"""
        print("=" * 70)
        print("STEP PIPE LENGTH CALCULATOR - UNIVERSAL (BSPLINE COMPATIBLE)")
        print("=" * 70)
        
        self.load_step()
        self.extract_centerline()
        
        if len(self.segments) > 0:
            self.merge_duplicates()
            self.total_length = sum(s["length"] for s in self.segments)
            self.display_results()
            self.build_centerline_wire()
            
            if export_step and self.centerline_wire:
                output = Path(self.step_file).stem + "_centerline.step"
                self.export_centerline_step(output)
        else:
            print("\n⚠ NO PIPE SEGMENTS DETECTED")
            print("File may not contain pipe geometry or is unsupported format.")
        
        return self.total_length
    
    def display_results(self):
        """Print results"""
        print("\n" + "=" * 70)
        print("RESULTS")
        print("=" * 70)
        print(f"Total Segments: {len(self.segments)}")
        
        for i, s in enumerate(self.segments, 1):
            seg_type = s["type"]
            length = s["length"]
            radius = s.get("radius", 0)
            source = s.get("source", "")
            
            if seg_type == "CIRCULAR_BEND":
                angle = s.get("angle_deg", 0)
                print(f"{i}. {seg_type:<18} {length:>10.2f} mm  R={radius:.1f}mm ∠={angle:.1f}° [{source}]")
            else:
                print(f"{i}. {seg_type:<18} {length:>10.2f} mm  R={radius:.1f}mm [{source}]")
        
        print("\n" + "=" * 70)
        print(f"TOTAL CENTERLINE LENGTH: {self.total_length:.2f} mm")
        print(f"                         {self.total_length/1000:.4f} meters")
        print("=" * 70)


def main():
    if len(sys.argv) < 2:
        print("Usage: python universal_pipe_calculator.py file.step [--export-step]")
        print("\nThis version handles:")
        print("  ✓ Analytic surfaces (Cylinder, Torus)")
        print("  ✓ BSpline/NURBS approximated surfaces")
        return
    
    step_file = sys.argv[1]
    export_step = "--export-step" in sys.argv
    
    calc = UniversalPipeLengthCalculator(Path(step_file))
    calc.calculate(export_step=export_step)


if __name__ == "__main__":
    main()
