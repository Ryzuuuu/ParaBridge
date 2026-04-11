"""
Bridge Model Generator
Builds a parametric 3D model of a steel girder bridge using pythonOCC.
"""
import sys
import math
import argparse
from pathlib import Path

# ==========================================
# PARAMETRIC DESIGN VARIABLES
# ==========================================
units = "mm"

# Geometry & layout
span_length_L = 12000.0
n_girders = 3
girder_centroid_spacing = 3000.0
deck_overhang = 500.0
# Derived deck width = (n_girders - 1) * girder_centroid_spacing + 2 * deck_overhang
deck_width = (n_girders - 1) * girder_centroid_spacing + 2 * deck_overhang

# Main girder
girder_section_d = 900.0
girder_section_bf = 300.0
girder_section_tf = 16.0
girder_section_tw = 10.0
girder_length = span_length_L

# Deck slab
deck_thickness = 200.0

# Pier & pier cap
pier_diameter = 800.0
pier_height = 3000.0
pier_cap_length = 1200.0
pier_cap_top_width = deck_width  # 7000.0
pier_cap_bottom_width = 800.0
pier_cap_depth = 600.0

# Pile & pile cap
n_piles_per_cap = 4
pile_diameter = 400.0
pile_length = 5000.0
pile_cap_length = 2200.0
pile_cap_width = 2200.0
pile_cap_depth = 600.0
pile_spacing = 900.0

# Reinforcement
rebar_main_diameter = 16.0
rebar_transverse_diameter = 8.0
rebar_spacing_longitudinal = 150.0
rebar_spacing_transverse = 200.0
rebar_cover = 40.0
rebar_visible = True
concrete_opacity = 0.35

# Visualization & export
show_axes = False
background_color = "grey"
save_step = True
step_filename = "output/bridge_model.step"


# ==========================================
# DEPENDENCIES
# ==========================================
from draw_i_section import create_i_section as raw_create_i_section
from draw_rectangular_prism import create_rectangular_prism as raw_create_prism

try:
    from OCC.Core.BRep import BRep_Builder
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_Transform
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism, BRepPrimAPI_MakeTorus
    from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
    from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Writer
    from OCC.Core.TopoDS import TopoDS_Compound
    from OCC.Core.AIS import AIS_Shape
    from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
    from OCC.Display.SimpleGui import init_display
except ImportError:
    print("Error: pythonOCC is not installed.")
    sys.exit(1)


# ==========================================
# FACTORY FUNCTIONS
# ==========================================

def move_shape(shape, dx=0.0, dy=0.0, dz=0.0):
    transform = gp_Trsf()
    transform.SetTranslation(gp_Vec(dx, dy, dz))
    return BRepBuilderAPI_Transform(shape, transform, True).Shape()

def create_circular_pier(diameter, height):
    axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    return BRepPrimAPI_MakeCylinder(axis, diameter / 2.0, height).Shape()

def create_trapezoidal_pier_cap(length, width_top, width_bottom, depth):
    # extrusion of trapezoid polygon
    w_top_half = width_top / 2.0
    w_bot_half = width_bottom / 2.0

    poly = BRepBuilderAPI_MakePolygon()
    poly.Add(gp_Pnt(0, -w_bot_half, 0))
    poly.Add(gp_Pnt(0, w_bot_half, 0))
    poly.Add(gp_Pnt(0, w_top_half, depth))
    poly.Add(gp_Pnt(0, -w_top_half, depth))
    poly.Close()

    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(length, 0, 0)).Shape()

def create_pile(diameter, length):
    axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    return BRepPrimAPI_MakeCylinder(axis, diameter / 2.0, length).Shape()

def create_pile_cap(length, width, depth):
    # using FOSSEE factory
    cap = raw_create_prism(length, width, depth)
    return move_shape(cap, dx=-length / 2.0, dy=-width / 2.0)

def create_i_section(d, bf, tf, tw, length):
    # wraps FOSSEE factory to match prompt arg lists
    return raw_create_i_section(length, bf, bf, d, tf, tf, tw)

def create_rectangular_prism(width, height, length):
    # wraps FOSSEE factory to match prompt arg lists (X=length, Y=width, Z=height)
    return raw_create_prism(length, width, height)

def create_rebar_grid_for_deck(deck_w, span, cover, diam, spacing):
    # models simple orthogonal rebar mesh inside deck
    bars = []
    margin = cover + (diam / 2.0)

    # Note: rebar logic returns cylinder shapes
    def make_bar(r, l, x, y, z, dx, dy):
        ax = gp_Ax2(gp_Pnt(x, y, z), gp_Dir(dx, dy, 0))
        return BRepPrimAPI_MakeCylinder(ax, r, l).Shape()

    # longitudinal bars
    length_x = span - 2 * cover
    start_x = -span / 2.0 + cover
    for z_pos in [margin, deck_thickness - margin]:
        curr_y = -deck_w / 2.0 + margin
        while curr_y <= deck_w / 2.0 - margin + 0.001:
            bars.append(make_bar(diam / 2, length_x, start_x, curr_y, z_pos, 1, 0))
            curr_y += spacing

    # transverse blocks 
    length_y = deck_w - 2 * cover
    start_y = -deck_w / 2.0 + cover
    for z_pos in [margin, deck_thickness - margin]:
        curr_x = -span / 2.0 + cover
        while curr_x <= span / 2.0 - cover + 0.001:
            bars.append(make_bar(diam / 2, length_y, curr_x, start_y, z_pos, 0, 1))
            curr_x += spacing

    return bars


# ==========================================
# ASSEMBLY FUNCTIONS
# ==========================================

def build_girders():
    print(f"Adding {n_girders} steel girders...")
    girders = []
    z_bottom = -(deck_thickness + girder_section_d)
    
    for i in range(n_girders):
        y_offset = (i - (n_girders - 1) / 2.0) * girder_centroid_spacing
        girder = create_i_section(girder_section_d, girder_section_bf, girder_section_tf, girder_section_tw, girder_length)
        girder = move_shape(girder, dx=-girder_length / 2.0, dy=y_offset, dz=z_bottom)
        girders.append(girder)
        
    return girders

def build_crossframes():
    print("Adding cross bracing...")
    braces = []
    z_bot_girder = -(deck_thickness + girder_section_d)
    radius = 15.0
    
    z_brace_bot = z_bot_girder + girder_section_tf
    z_brace_top = z_bot_girder + girder_section_d - girder_section_tf

    for i in range(n_girders - 1):
        y_left_girder = (i - (n_girders - 1) / 2.0) * girder_centroid_spacing
        y_right = y_left_girder + girder_centroid_spacing - girder_section_tw
        y_left = y_left_girder + girder_section_tw / 2.0
        
        dy = y_right - y_left
        dz = z_brace_top - z_brace_bot
        length = math.sqrt(dy**2 + dz**2)

        for j in range(1, 6):
            x = -span_length_L / 2.0 + j * (span_length_L / 6.0)
            ax1 = gp_Ax2(gp_Pnt(x, y_left, z_brace_bot), gp_Dir(0, dy, dz))
            b1 = BRepPrimAPI_MakeCylinder(ax1, radius, length).Shape()
            
            ax2 = gp_Ax2(gp_Pnt(x, y_right, z_brace_bot), gp_Dir(0, -dy, dz))
            b2 = BRepPrimAPI_MakeCylinder(ax2, radius, length).Shape()
            
            braces.extend([b1, b2])
            
    return braces

def build_deck():
    print("Making the concrete deck...")
    deck = create_rectangular_prism(deck_width, deck_thickness, span_length_L)
    deck = move_shape(deck, dx=-span_length_L / 2.0, dy=-deck_width / 2.0, dz=-deck_thickness)
    return deck

def build_piers_and_pilecaps():
    print("Building substructure and foundations...")
    z_deck = -(deck_thickness + girder_section_d)
    z_cap_bot = z_deck - pier_cap_depth
    z_pier_bot = z_cap_bot - pier_height
    z_pcap_bot = z_pier_bot - pile_cap_depth

    # Piers placed exactly 500mm inside span
    pier_x_coords = [-span_length_L / 2.0 + 500, span_length_L / 2.0 - 500]
    
    pier_caps, piers, pile_caps, piles, stirrups = [], [], [], [], []

    for px in pier_x_coords:
        cap = create_trapezoidal_pier_cap(pier_cap_length, pier_cap_top_width, pier_cap_bottom_width, pier_cap_depth)
        cap = move_shape(cap, dx=px - pier_cap_length / 2.0, dz=z_cap_bot)
        pier_caps.append(cap)

        pier = create_circular_pier(pier_diameter, pier_height)
        pier = move_shape(pier, dx=px, dz=z_pier_bot)
        piers.append(pier)

        pcap = create_pile_cap(pile_cap_length, pile_cap_width, pile_cap_depth)
        pcap = move_shape(pcap, dx=px, dz=z_pcap_bot)
        pile_caps.append(pcap)

        offset = pile_spacing / 2.0
        pile_locations = [(-offset, -offset), (-offset, offset), (offset, -offset), (offset, offset)]
        
        for ox, oy in pile_locations:
            z_pile_bot = z_pcap_bot - pile_length
            pile = create_pile(pile_diameter, pile_length)
            pile = move_shape(pile, dx=px + ox, dy=oy, dz=z_pile_bot)
            piles.append(pile)

            z_ring = 500.0
            major_r = (pile_diameter / 2.0) + 5.0
            minor_r = 5.0
            while z_ring < pile_length - 250:
                ax = gp_Ax2(gp_Pnt(0, 0, z_ring), gp_Dir(0, 0, 1))
                ring = BRepPrimAPI_MakeTorus(ax, major_r, minor_r).Shape()
                ring = move_shape(ring, dx=px + ox, dy=oy, dz=z_pile_bot)
                stirrups.append(ring)
                z_ring += 500.0

    return {
        "pier_caps": pier_caps,
        "piers": piers,
        "pile_caps": pile_caps,
        "piles": piles,
        "pile_stirrups": stirrups
    }

def assemble_bridge():
    print("--- Starting bridge assembly ---")
    girders = build_girders()
    crossframes = build_crossframes()
    deck = build_deck()
    sub = build_piers_and_pilecaps()
    
    rebars = []
    if rebar_visible:
        rebars = create_rebar_grid_for_deck(deck_width, span_length_L, rebar_cover, rebar_main_diameter, rebar_spacing_longitudinal)
        rebars = [move_shape(b, dz=-deck_thickness) for b in rebars]

    parts = {
        "girders": girders,
        "crossframes": crossframes,
        "deck": [deck],
        "pier_caps": sub["pier_caps"],
        "piers": sub["piers"],
        "pile_caps": sub["pile_caps"],
        "piles": sub["piles"],
        "pile_stirrups": sub["pile_stirrups"],
        "rebars": rebars,
    }

    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)

    def add_item(obj):
        if hasattr(obj, "ShapeType"):
            builder.Add(compound, obj)
        elif isinstance(obj, list):
            for i in obj:
                add_item(i)

    for items in parts.values():
        add_item(items)

    print("Assembly complete!")
    return compound, parts

def launch_viewer(parts):
    print("Opening 3D viewer...")
    display, start, _, _ = init_display(size=(1000, 800))
    
    if background_color == "grey":
        display.set_bg_gradient_color([210, 210, 210], [250, 250, 250])

    def render(shapes, r, g, b, alpha=0.0):
        color = Quantity_Color(r, g, b, Quantity_TOC_RGB)
        lst = shapes if isinstance(shapes, list) else [shapes]
        for s in lst:
            if hasattr(s, "ShapeType"):
                ais = AIS_Shape(s)
                ais.SetDisplayMode(1)
                ais.SetColor(color)
                if alpha > 0:
                    ais.SetTransparency(alpha)
                display.Context.Display(ais, False)

    render(parts["girders"], 0.25, 0.25, 0.35)
    render(parts["crossframes"], 0.25, 0.25, 0.35)
    render(parts["rebars"], 1.0, 0.5, 0.0)
    render(parts["deck"], 0.75, 0.75, 0.75, concrete_opacity)
    
    subs = [parts["pier_caps"], parts["piers"], parts["pile_caps"], parts["piles"], parts["pile_stirrups"]]
    for item in subs:
        render(item, 0.5, 0.8, 0.9, concrete_opacity)

    display.Context.UpdateCurrentViewer()
    
    if show_axes:
        try:
            display.display_triedron()
        except Exception:
            pass

    display.View_Iso()
    display.FitAll()
    start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--span", type=float, help="Bridge span length")
    args = parser.parse_args()

    # CLI override
    global span_length_L
    if args.span:
        span_length_L = args.span

    comp, parts = assemble_bridge()

    if save_step:
        print(f"Exporting model to {step_filename}...")
        Path(step_filename).parent.mkdir(parents=True, exist_ok=True)
        writer = STEPControl_Writer()
        writer.Transfer(comp, STEPControl_AsIs)
        writer.Write(step_filename)

    launch_viewer(parts)


if __name__ == "__main__":
    main()
