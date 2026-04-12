"""
ParaBridge — Parametric Steel Girder Bridge CAD Model
======================================================
Generates a fully parametric, FOSSEE-compliant 3D model of a short-span
steel girder bridge using pythonOCC (pythonocc-core).

Coordinate system
-----------------
  X-axis → Longitudinal (span direction)
  Y-axis → Transverse  (deck width)
  Z-axis → Vertical    (upward positive)
  Origin → Centre of span at top surface of deck slab

Units : millimetres by default.  Pass units="m" or --units m to work in metres.
"""

import sys
import math
import argparse
from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------------------
# SECTION 1 — PARAMETRIC DESIGN VARIABLES
# ---------------------------------------------------------------------------

@dataclass
class BridgeConfig:
    # ── Units ─────────────────────────────────────────────────────────────
    units: str = "mm"                    # "mm" or "m"

    # ── Geometry & layout ─────────────────────────────────────────────────
    span_length_L: float          = 12000.0   # clear span between pier faces
    n_girders: int                = 3          # number of main longitudinal girders (≥ 3)
    girder_centroid_spacing: float= 3000.0    # centre-to-centre girder spacing
    girder_offset_from_edge: float= 500.0     # deck overhang beyond outermost girder
    n_crossframes: int            = 5          # number of cross-frame planes along span
    pier_location_x: float        = 5500.0    # |X| from span centre for each pier (symmetric pair)
    n_lanes: int                  = 2          # lane count (informational / future demarcation)
    lane_width: float             = 3500.0    # width per traffic lane

    # ── Main girder — I-section (IS 800 / IS 2062) ───────────────────────
    girder_section_d: float       = 900.0     # total section depth
    girder_section_bf_bot: float  = 300.0     # bottom (tension) flange width
    girder_section_bf_top: float  = 300.0     # top (compression) flange width
    girder_section_tf_bot: float  = 16.0      # bottom flange thickness
    girder_section_tf_top: float  = 16.0      # top flange thickness
    girder_section_tw: float      = 10.0      # web thickness
    girder_material: str          = "Fe410"   # steel material grade (IS 2062)

    # ── Deck slab ─────────────────────────────────────────────────────────
    deck_thickness: float         = 200.0
    deck_slab_segment_length: float = 3000.0  # prism segment length for deck modelling
    deck_cover: float             = 40.0       # concrete cover for deck rebar
    deck_material_opacity: float  = 0.35       # semi-transparency of deck concrete

    # ── Pier & pier cap ───────────────────────────────────────────────────
    pier_diameter: float          = 800.0
    pier_height: float            = 3000.0
    pier_cap_length: float        = 1200.0     # longitudinal (X) extent of pier cap
    # pier_cap_top_width: left as None → auto-set in __post_init__ to span all girders
    pier_cap_top_width: float     = 0.0        # transverse width at cap top (0 = auto)
    pier_cap_bottom_width: float  = 0.0        # transverse width at cap bottom (0 = auto = pier dia + 2×200)
    pier_cap_depth: float         = 600.0
    pier_cap_elevation: float     = 0.0        # Z-offset applied to computed pier-cap position
    pier_n_long_bars: int         = 8          # longitudinal rebar count in circular pier

    # ── Pile & pile cap ───────────────────────────────────────────────────
    n_piles_per_cap: int          = 4
    pile_diameter: float          = 400.0
    pile_length: float            = 5000.0
    pile_spacing: float           = 1200.0     # centre-to-centre spacing between pile pairs
    pile_cap_length: float        = 2200.0
    pile_cap_width: float         = 2200.0     # match pile_cap_length for a square cap
    pile_cap_depth: float         = 800.0      # deeper cap for better visual connection
    pile_cap_elevation: float     = 0.0        # Z-offset applied to computed pile-cap position

    # ── Reinforcement (common) ────────────────────────────────────────────
    rebar_main_diameter: float      = 16.0
    rebar_transverse_diameter: float= 8.0
    rebar_spacing_longitudinal: float = 150.0
    rebar_spacing_transverse: float   = 200.0
    rebar_cover: float              = 40.0
    rebar_visible: bool             = True
    concrete_opacity: float         = 0.35     # opacity for pier / pile concrete

    # ── Pile stirrups ─────────────────────────────────────────────────────
    pile_rebar_diameter: float    = 5.0
    pile_rebar_spacing: float     = 500.0
    pile_lod_enabled: bool        = False      # True → lightweight edge rings; False → torus solids

    # ── Cross-frame members ────────────────────────────────────────────────
    crossframe_radius: float      = 15.0       # radius of cylindrical cross-frame members

    # ── Visualization & export ────────────────────────────────────────────
    show_axes: bool               = False
    background_color: str         = "grey"
    save_step: bool               = True
    step_filename: str            = "output/bridge_model.step"
    save_brep: bool               = False
    brep_filename: str            = "output/bridge_model.brep"
    render_size: tuple            = (1200, 800)

    # ── Auto-derived fields (do not set manually) ──────────────────────────
    unit_scale: float  = field(init=False)
    deck_width: float  = field(init=False)
    girder_length: float = field(init=False)

    def __post_init__(self):
        # Compute unit_scale for reference (used by tests, but NOT applied to dims)
        # All dimensional inputs are already in correct units (mm for mm, already-converted for m)
        self.unit_scale = 0.001 if str(self.units).strip().lower() == "m" else 1.0

        # Integer coercions
        self.n_girders         = int(self.n_girders)
        self.n_piles_per_cap   = int(self.n_piles_per_cap)
        self.n_crossframes     = int(self.n_crossframes)
        self.n_lanes           = int(self.n_lanes)
        self.pier_n_long_bars  = int(self.pier_n_long_bars)

        # Boolean coercions
        self.rebar_visible    = bool(self.rebar_visible)
        self.pile_lod_enabled = bool(self.pile_lod_enabled)
        self.show_axes        = bool(self.show_axes)
        self.save_step        = bool(self.save_step)
        self.save_brep        = bool(self.save_brep)

        # Float coercions
        self.concrete_opacity      = float(self.concrete_opacity)
        self.deck_material_opacity = float(self.deck_material_opacity)

        # Derived geometry
        self.deck_width   = (self.n_girders - 1) * self.girder_centroid_spacing \
                            + 2.0 * self.girder_offset_from_edge
        self.girder_length = self.span_length_L

        # Auto-derive pier cap widths when sentinel 0.0 is left unchanged:
        #   top_width  → spans all girders + one pier_diameter overhang per side
        #   bottom_width → pier diameter + 2 × 200 mm bearing ledge
        outer_girder_span = (self.n_girders - 1) * self.girder_centroid_spacing
        if self.pier_cap_top_width == 0.0:
            self.pier_cap_top_width = outer_girder_span + 2.0 * self.pier_diameter
        if self.pier_cap_bottom_width == 0.0:
            self.pier_cap_bottom_width = self.pier_diameter + 2.0 * 200.0


# ---------------------------------------------------------------------------
# SECTION 2 — OCC IMPORTS
# ---------------------------------------------------------------------------

try:
    from OCC.Core.BRep import BRep_Builder
    from OCC.Core.BRepBuilderAPI import (
        BRepBuilderAPI_MakeFace,
        BRepBuilderAPI_MakePolygon,
        BRepBuilderAPI_MakeEdge,
        BRepBuilderAPI_Transform,
    )
    from OCC.Core.BRepPrimAPI import (
        BRepPrimAPI_MakeBox,
        BRepPrimAPI_MakeCylinder,
        BRepPrimAPI_MakePrism,
        BRepPrimAPI_MakeTorus,
    )
    from OCC.Core.BRepTools import breptools_Write
    from OCC.Core.Geom import Geom_Circle
    from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
    from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
    from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Writer
    from OCC.Core.TopoDS import TopoDS_Compound
    from OCC.Core.AIS import AIS_Shape
    from OCC.Display.SimpleGui import init_display

    from draw_i_section import create_i_section as _raw_i_section
    from draw_rectangular_prism import create_rectangular_prism as _raw_prism

except ImportError as exc:
    print(f"Import error — ensure pythonOCC is installed in your conda env.\nDetail: {exc}")
    sys.exit(1)


# ---------------------------------------------------------------------------
# SECTION 3 — LOW-LEVEL GEOMETRY UTILITIES
# ---------------------------------------------------------------------------

def _translate(shape, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0):
    """Return a translated copy of *shape* without modifying the original."""
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(dx, dy, dz))
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def _make_compound(shapes: list) -> TopoDS_Compound:
    """Pack an arbitrary list of TopoDS shapes into a single Compound."""
    builder  = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for s in shapes:
        if s is not None and hasattr(s, "ShapeType"):
            builder.Add(compound, s)
    return compound


def _cylinder(radius: float, height: float,
              origin: tuple = (0, 0, 0),
              direction: tuple = (0, 0, 1)) -> object:
    """Create a solid cylinder with arbitrary axis."""
    ax = gp_Ax2(gp_Pnt(*origin), gp_Dir(*direction))
    return BRepPrimAPI_MakeCylinder(ax, radius, height).Shape()


# ---------------------------------------------------------------------------
# SECTION 4 — COMPONENT FACTORY FUNCTIONS
# ---------------------------------------------------------------------------

def create_i_section(d: float, bf_bot: float, bf_top: float,
                     tf_bot: float, tf_top: float,
                     tw: float, length: float):
    """
    Create an asymmetric steel I-section of the given *length*.

    Wraps draw_i_section.create_i_section, exposing separate top and bottom
    flange dimensions so composite bridge sections can be accurately modelled.

    Parameters
    ----------
    d       : total section depth
    bf_bot  : bottom (tension) flange width
    bf_top  : top (compression / composite) flange width
    tf_bot  : bottom flange thickness
    tf_top  : top flange thickness
    tw      : web thickness
    length  : extrusion length (= girder span)
    """
    return _raw_i_section(length, bf_top, bf_bot, d, tf_top, tf_bot, tw)


def create_rectangular_prism(width: float, height: float, length: float):
    """Thin wrapper around the FOSSEE rectangular prism primitive."""
    return _raw_prism(length, width, height)


def create_circular_pier(diameter: float, height: float):
    """Solid cylinder representing a circular RC pier column."""
    return _cylinder(diameter / 2.0, height)


def create_trapezoidal_pier_cap(length: float, width_top: float,
                                width_bottom: float, depth: float):
    """
    Hammerhead pier cap with a trapezoidal cross-section.
    The taper transitions over the lower half of the cap depth.
    """
    half_top = width_top    / 2.0
    half_bot = width_bottom / 2.0
    taper_z  = depth * 0.5

    poly = BRepBuilderAPI_MakePolygon()
    for pt in [
        (0, -half_bot, 0),
        (0,  half_bot, 0),
        (0,  half_top, taper_z),
        (0,  half_top, depth),
        (0, -half_top, depth),
        (0, -half_top, taper_z),
    ]:
        poly.Add(gp_Pnt(*pt))
    poly.Close()

    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(length, 0, 0)).Shape()


def create_pile(diameter: float, length: float):
    """Solid cylinder representing a bored / driven pile."""
    return _cylinder(diameter / 2.0, length)


def create_pile_cap(length: float, width: float, depth: float):
    """Rectangular solid pile cap, centred at XY origin."""
    cap = _raw_prism(length, width, depth)
    return _translate(cap, dx=-length / 2.0, dy=-width / 2.0)


def create_rebar_grid_for_deck(deck_w: float, span: float,
                                cover: float, diam: float,
                                long_spacing: float, trans_spacing: float,
                                deck_thickness: float) -> list:
    """
    Two layers (top & bottom) of a full orthogonal rebar grid inside the deck slab.

    Longitudinal bars run along X; transverse bars run along Y.
    Both layers respect the specified concrete cover on all faces.
    """
    bars   = []
    bar_r  = diam / 2.0
    margin = cover + bar_r

    # Helper — one cylindrical rebar along a given direction
    def _bar(length, origin, direction):
        return _cylinder(bar_r, length, origin, direction)

    long_len  = span  - 2.0 * cover
    trans_len = deck_w - 2.0 * cover
    start_x   = -span   / 2.0 + cover
    start_y   = -deck_w / 2.0 + cover

    for z in [margin, deck_thickness - margin]:          # bottom layer then top layer
        # Longitudinal bars (along X), one per Y position
        y = -deck_w / 2.0 + margin
        while y <= deck_w / 2.0 - margin + 1e-3:
            bars.append(_bar(long_len, (start_x, y, z), (1, 0, 0)))
            y += long_spacing

        # Transverse bars (along Y), one per X position
        x = -span / 2.0 + cover
        while x <= span / 2.0 - cover + 1e-3:
            bars.append(_bar(trans_len, (x, start_y, z), (0, 1, 0)))
            x += trans_spacing

    return bars


def create_rebar_for_circular_section(n_bars: int, host_radius: float,
                                       bar_diam: float, cover: float,
                                       height: float) -> list:
    """
    Longitudinal reinforcement bars arranged in a ring inside a circular column.

    Parameters
    ----------
    n_bars      : number of vertical bars (evenly spaced around the circumference)
    host_radius : outer radius of the concrete column
    bar_diam    : diameter of each longitudinal bar
    cover       : concrete cover to bar face
    height      : bar length (= column height)
    """
    ring_r = host_radius - cover - bar_diam / 2.0
    bars   = []
    for k in range(n_bars):
        angle = 2.0 * math.pi * k / n_bars
        bx    = ring_r * math.cos(angle)
        by    = ring_r * math.sin(angle)
        bars.append(_cylinder(bar_diam / 2.0, height, (bx, by, 0), (0, 0, 1)))
    return bars


def create_rebar_for_rect_section(sect_length: float, sect_width: float,
                                   sect_depth: float,
                                   bar_diam: float, cover: float,
                                   long_spacing: float) -> list:
    """
    Top-and-bottom longitudinal rebar layers inside a rectangular beam / cap.

    Bars run along the *length* axis (X), spaced at *long_spacing* across *width* (Y).
    """
    bars   = []
    bar_r  = bar_diam / 2.0
    margin = cover + bar_r
    bar_l  = sect_length - 2.0 * cover

    for z in [margin, sect_depth - margin]:
        y = -sect_width / 2.0 + margin
        while y <= sect_width / 2.0 - margin + 1e-3:
            bars.append(_cylinder(bar_r, bar_l,
                                  (-sect_length / 2.0 + cover, y, z),
                                  (1, 0, 0)))
            y += long_spacing
    return bars


def create_pile_stirrups(pile_x: float, pile_y: float, pile_z_bot: float,
                          pile_len: float, pile_diam: float,
                          stirrup_diam: float, stirrup_spacing: float,
                          use_proxy: bool = False) -> list:
    """
    Circular stirrup rings inside a pile, placed at regular vertical intervals.

    use_proxy=True  → lightweight edge circles (fast export / LOD)
    use_proxy=False → solid torus rings (full geometry)
    """
    rings   = []
    major_r = pile_diam / 2.0 + stirrup_diam / 2.0
    minor_r = stirrup_diam / 2.0
    z       = stirrup_spacing / 2.0

    while z < pile_len - stirrup_spacing / 2.0:
        if use_proxy:
            circle = Geom_Circle(
                gp_Ax2(gp_Pnt(pile_x, pile_y, pile_z_bot + z), gp_Dir(0, 0, 1)),
                major_r,
            )
            rings.append(BRepBuilderAPI_MakeEdge(circle).Edge())
        else:
            ax   = gp_Ax2(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1))
            ring = BRepPrimAPI_MakeTorus(ax, major_r, minor_r).Shape()
            rings.append(_translate(ring, dx=pile_x, dy=pile_y, dz=pile_z_bot))
        z += stirrup_spacing

    return rings


# ---------------------------------------------------------------------------
# SECTION 5 — ASSEMBLY FUNCTIONS
# ---------------------------------------------------------------------------

def build_girders(config: BridgeConfig) -> list:
    """
    Fabricate and position *n_girders* steel I-section girders.

    Girders sit immediately beneath the deck slab, spanning the full clear span.
    Their bottom flanges are at Z = -(deck_thickness + girder_section_d).
    """
    print(f"  → Placing {config.n_girders} steel girders [{config.girder_material}] ...")
    girders = []
    z_bot   = -(config.deck_thickness + config.girder_section_d)

    for i in range(config.n_girders):
        y_offset = (i - (config.n_girders - 1) / 2.0) * config.girder_centroid_spacing
        girder   = create_i_section(
            config.girder_section_d,
            config.girder_section_bf_bot,
            config.girder_section_bf_top,
            config.girder_section_tf_bot,
            config.girder_section_tf_top,
            config.girder_section_tw,
            config.girder_length,
        )
        girder = _translate(girder,
                            dx=-config.girder_length / 2.0,
                            dy=y_offset,
                            dz=z_bot)
        girders.append(girder)

    return girders


def build_crossframes(config: BridgeConfig) -> list:
    """
    Place diagonal cross-frame bracing between adjacent girder pairs.

    The number of cross-frame planes along the span is controlled by
    *n_crossframes*.  Each plane contains two diagonal members (X-pattern)
    per inter-girder bay.
    """
    print(f"  → Adding {config.n_crossframes} cross-frame planes ...")
    braces  = []
    z_bot_g = -(config.deck_thickness + config.girder_section_d)
    z_bot_b = z_bot_g + config.girder_section_tf_bot
    z_top_b = z_bot_g + config.girder_section_d - config.girder_section_tf_top
    r       = config.crossframe_radius

    # X positions for each cross-frame plane (evenly distributed, excluding end-faces)
    step = config.span_length_L / (config.n_crossframes + 1)
    x_positions = [-config.span_length_L / 2.0 + step * (j + 1)
                   for j in range(config.n_crossframes)]

    for i in range(config.n_girders - 1):
        y_l = (i     - (config.n_girders - 1) / 2.0) * config.girder_centroid_spacing \
              + config.girder_section_tw / 2.0
        y_r = y_l + config.girder_centroid_spacing - config.girder_section_tw

        dy = y_r - y_l
        dz = z_top_b - z_bot_b
        length = math.hypot(dy, dz)

        for xp in x_positions:
            # Bottom-left → top-right diagonal
            b1 = _cylinder(r, length, (xp, y_l, z_bot_b), (0, dy, dz))
            # Bottom-right → top-left diagonal
            b2 = _cylinder(r, length, (xp, y_r, z_bot_b), (0, -dy, dz))
            braces.extend([b1, b2])

    return braces


def build_deck(config: BridgeConfig) -> list:
    """
    Construct the RC deck slab as a series of equal-length prism segments.

    The number of segments is determined by *deck_slab_segment_length*.
    The deck top surface sits at Z = 0; its bottom at Z = -deck_thickness.
    """
    print("  → Constructing deck slab segments ...")
    n_segs  = max(1, math.ceil(config.span_length_L / config.deck_slab_segment_length))
    seg_len = config.span_length_L / n_segs
    segments = []

    x_start = -config.span_length_L / 2.0

    for k in range(n_segs):
        seg = create_rectangular_prism(config.deck_width, config.deck_thickness, seg_len)
        seg = _translate(seg,
                         dx=x_start + k * seg_len,
                         dy=-config.deck_width / 2.0,
                         dz=-config.deck_thickness)
        segments.append(seg)

    return segments


def build_piers_and_pilecaps(config: BridgeConfig) -> dict:
    """
    Construct the complete bridge substructure:

    * Pier caps  (trapezoidal hammerhead)
    * Pier columns  (circular)
    * Pile caps  (rectangular)
    * Piles  (circular)
    * Pile stirrup rings  (torus or edge proxy)

    Piers are located symmetrically at ±pier_location_x from span centre.
    pier_cap_elevation and pile_cap_elevation allow independent Z adjustment.
    """
    print("  → Building substructure and foundations ...")

    # Reference elevations (from deck-top = Z=0, downward is negative)
    #
    # Stacking order (top to bottom):
    #   deck top     Z = 0
    #   deck bottom  Z = -deck_thickness
    #   girder bottom Z = -(deck_thickness + girder_section_d)
    #   pier cap top  Z = -(deck_thickness + girder_section_d)   ← flush under girder bottom
    #   pier cap bot  Z = -(deck_thickness + girder_section_d) - pier_cap_depth
    #   pier top      Z = pier cap bottom  (pier sits directly under cap)
    #   pier bottom   Z = pier cap bottom - pier_height
    #   pile cap top  Z = pier bottom      (pile cap sits directly under pier)
    #   pile cap bot  Z = pier bottom - pile_cap_depth
    #   pile tops     Z = pile cap bottom
    #   pile bottoms  Z = pile cap bottom - pile_length

    z_girder_bot = -(config.deck_thickness + config.girder_section_d)
    z_cap_top    = z_girder_bot + config.pier_cap_elevation          # pier cap top flush under girder bottom
    z_cap_bot    = z_cap_top - config.pier_cap_depth                 # pier cap bottom
    z_pier_top   = z_cap_bot                                          # pier column top
    z_pier_bot   = z_pier_top - config.pier_height                   # pier column bottom
    z_pcap_top   = z_pier_bot                                         # pile cap top
    z_pcap_bot   = z_pcap_top - config.pile_cap_depth

    pier_xs = [-config.pier_location_x, config.pier_location_x]

    pier_caps, piers, pile_caps, piles, stirrups = [], [], [], [], []

    for px in pier_xs:
        # — Pier cap —
        cap = create_trapezoidal_pier_cap(
            config.pier_cap_length,
            config.pier_cap_top_width,
            config.pier_cap_bottom_width,
            config.pier_cap_depth,
        )
        cap = _translate(cap, dx=px - config.pier_cap_length / 2.0, dz=z_cap_bot)
        pier_caps.append(cap)

        # — Pier column —
        pier = create_circular_pier(config.pier_diameter, config.pier_height)
        pier = _translate(pier, dx=px, dz=z_pier_bot)       # bottom of column
        piers.append(pier)

        # — Pile cap —
        pcap = create_pile_cap(
            config.pile_cap_length,
            config.pile_cap_width,
            config.pile_cap_depth,
        )
        pcap = _translate(pcap, dx=px, dz=z_pcap_bot)
        pile_caps.append(pcap)

        # — Piles (2 × 2 grid) —
        # Longitudinal offset (ox) uses pile_cap_length / 4 so piles
        # sit comfortably within the pile cap footprint in both axes.
        ox_step = config.pile_cap_length / 4.0
        oy_step = config.pile_spacing    / 2.0
        pile_offsets = [
            (-ox_step, -oy_step), (-ox_step,  oy_step),
            ( ox_step, -oy_step), ( ox_step,  oy_step),
        ]
        z_pile_bot = z_pcap_bot - config.pile_length
        for ox, oy in pile_offsets:
            pile = create_pile(config.pile_diameter, config.pile_length)
            pile = _translate(pile, dx=px + ox, dy=oy, dz=z_pile_bot)
            piles.append(pile)

            pile_stirrups = create_pile_stirrups(
                pile_x=px + ox,
                pile_y=oy,
                pile_z_bot=z_pile_bot,
                pile_len=config.pile_length,
                pile_diam=config.pile_diameter,
                stirrup_diam=config.pile_rebar_diameter,
                stirrup_spacing=config.pile_rebar_spacing,
                use_proxy=config.pile_lod_enabled,
            )
            stirrups.extend(pile_stirrups)

    return {
        "pier_caps":     pier_caps,
        "piers":         piers,
        "pile_caps":     pile_caps,
        "piles":         piles,
        "pile_stirrups": stirrups,
    }


def build_all_rebar(config: BridgeConfig, sub: dict) -> dict:
    """
    Generate steel reinforcement for every concrete component.

    Returns a dict with keys matching each concrete group so the viewer
    can render them with the correct steel colour independently.
    """
    if not config.rebar_visible:
        return {k: [] for k in
                ["deck_rebars", "pier_rebars", "pier_cap_rebars", "pile_cap_rebars"]}

    print("  → Placing reinforcement bars ...")

    # ── Deck rebar grid ────────────────────────────────────────────────────
    deck_bars_raw = create_rebar_grid_for_deck(
        config.deck_width,
        config.span_length_L,
        config.deck_cover,
        config.rebar_main_diameter,
        config.rebar_spacing_longitudinal,
        config.rebar_spacing_transverse,
        config.deck_thickness,
    )
    # Shift so bars sit inside the deck slab (deck top = Z=0)
    deck_bars = [_translate(b, dz=-config.deck_thickness) for b in deck_bars_raw]

    # ── Pier longitudinal bars ─────────────────────────────────────────────
    pier_bars  = []
    z_girder_bot = -(config.deck_thickness + config.girder_section_d)
    z_cap_top    = z_girder_bot + config.pier_cap_elevation
    z_cap_bot    = z_cap_top - config.pier_cap_depth
    z_pier_bot   = z_cap_bot - config.pier_height

    for px in [-config.pier_location_x, config.pier_location_x]:
        raw_bars = create_rebar_for_circular_section(
            config.pier_n_long_bars,
            config.pier_diameter / 2.0,
            config.rebar_main_diameter,
            config.rebar_cover,
            config.pier_height,
        )
        pier_bars.extend([_translate(b, dx=px, dz=z_pier_bot) for b in raw_bars])

    # ── Pier cap rebar ─────────────────────────────────────────────────────
    pier_cap_bars = []
    for px in [-config.pier_location_x, config.pier_location_x]:
        raw_bars = create_rebar_for_rect_section(
            config.pier_cap_length,
            config.pier_cap_bottom_width,   # conservative — narrowest width
            config.pier_cap_depth,
            config.rebar_main_diameter,
            config.rebar_cover,
            config.rebar_spacing_longitudinal,
        )
        pier_cap_bars.extend(
            [_translate(b, dx=px, dz=z_cap_bot) for b in raw_bars]
        )

    # ── Pile cap rebar ─────────────────────────────────────────────────────
    pile_cap_bars = []
    z_pcap_top = z_pier_bot
    z_pcap_bot = z_pcap_top - config.pile_cap_depth

    for px in [-config.pier_location_x, config.pier_location_x]:
        raw_bars = create_rebar_for_rect_section(
            config.pile_cap_length,
            config.pile_cap_width,
            config.pile_cap_depth,
            config.rebar_main_diameter,
            config.rebar_cover,
            config.rebar_spacing_longitudinal,
        )
        pile_cap_bars.extend([_translate(b, dx=px, dz=z_pcap_bot) for b in raw_bars])

    return {
        "deck_rebars":      deck_bars,
        "pier_rebars":      pier_bars,
        "pier_cap_rebars":  pier_cap_bars,
        "pile_cap_rebars":  pile_cap_bars,
    }


def assemble_bridge(config: BridgeConfig):
    """
    Orchestrate all component builders and pack everything into one
    TopoDS_Compound suitable for STEP / BREP export.

    Returns
    -------
    compound : TopoDS_Compound  — full bridge topology
    parts    : dict             — keyed component lists for selective display
    """
    print("─" * 50)
    print("  ParaBridge — Assembly started")
    print("─" * 50)

    girders     = build_girders(config)
    crossframes = build_crossframes(config)
    deck_segs   = build_deck(config)
    sub         = build_piers_and_pilecaps(config)
    rebar       = build_all_rebar(config, sub)

    parts = {
        "girders":         girders,
        "crossframes":     crossframes,
        "deck":            deck_segs,
        "pier_caps":       sub["pier_caps"],
        "piers":           sub["piers"],
        "pile_caps":       sub["pile_caps"],
        "piles":           sub["piles"],
        "pile_stirrups":   sub["pile_stirrups"],
        "deck_rebars":     rebar["deck_rebars"],
        "pier_rebars":     rebar["pier_rebars"],
        "pier_cap_rebars": rebar["pier_cap_rebars"],
        "pile_cap_rebars": rebar["pile_cap_rebars"],
    }

    # Flatten everything into one compound for export
    all_shapes = []
    for group in parts.values():
        if isinstance(group, list):
            all_shapes.extend(group)
        elif hasattr(group, "ShapeType"):
            all_shapes.append(group)

    compound = _make_compound(all_shapes)

    print("─" * 50)
    print("  Assembly complete.")
    print("─" * 50)
    return compound, parts


# ---------------------------------------------------------------------------
# SECTION 6 — EXPORT
# ---------------------------------------------------------------------------

def export_step(compound, filename: str):
    """Write the assembly to an ISO 10303 STEP file."""
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    writer = STEPControl_Writer()
    writer.Transfer(compound, STEPControl_AsIs)
    status = writer.Write(filename)
    print(f"  STEP export → {filename}  (status {status})")


def export_brep(compound, filename: str):
    """Write the assembly to an OpenCASCADE BREP file."""
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    breptools_Write(compound, filename)
    print(f"  BREP export → {filename}")


# ---------------------------------------------------------------------------
# SECTION 7 — 3D VIEWER
# ---------------------------------------------------------------------------

def launch_viewer(config: BridgeConfig, parts: dict):
    """
    Open the interactive OCC 3D viewer and render all bridge components.

    Rendering scheme
    ----------------
    Steel (girders, crossframes, all rebar, pile stirrups) → opaque, steel-blue / orange
    Concrete (deck, piers, pier caps, pile caps, piles)    → semi-transparent
    """
    print("  Opening 3D viewer …")
    display, start, _, _ = init_display(size=config.render_size)

    if config.background_color == "grey":
        display.set_bg_gradient_color([200, 200, 200], [245, 245, 245])

    def _render(shapes, r, g, b, alpha=0.0):
        """Display a list of shapes with the given RGB colour and transparency."""
        color = Quantity_Color(r, g, b, Quantity_TOC_RGB)
        items = shapes if isinstance(shapes, list) else [shapes]
        for shape in items:
            if shape is None or not hasattr(shape, "ShapeType"):
                continue
            ais = AIS_Shape(shape)
            ais.SetDisplayMode(1)      # shaded
            ais.SetColor(color)
            if alpha > 0.0:
                ais.SetTransparency(alpha)
            display.Context.Display(ais, False)

    # ── Steel components — fully opaque ────────────────────────────────────
    steel_blue  = (0.25, 0.30, 0.45)
    rebar_amber = (0.85, 0.45, 0.05)

    _render(parts["girders"],         *steel_blue)
    _render(parts["crossframes"],     *steel_blue)
    _render(parts["deck_rebars"],     *rebar_amber)
    _render(parts["pier_rebars"],     *rebar_amber)
    _render(parts["pier_cap_rebars"], *rebar_amber)
    _render(parts["pile_cap_rebars"], *rebar_amber)
    _render(parts["pile_stirrups"],   *rebar_amber)    # ← steel, NOT concrete

    # ── Concrete components — semi-transparent ─────────────────────────────
    deck_grey    = (0.78, 0.78, 0.78)
    concrete_tan = (0.72, 0.68, 0.60)

    _render(parts["deck"],      *deck_grey,    alpha=config.deck_material_opacity)
    _render(parts["pier_caps"], *concrete_tan, alpha=config.concrete_opacity)
    _render(parts["piers"],     *concrete_tan, alpha=config.concrete_opacity)
    _render(parts["pile_caps"], *concrete_tan, alpha=config.concrete_opacity)
    _render(parts["piles"],     *concrete_tan, alpha=config.concrete_opacity)

    display.Context.UpdateCurrentViewer()

    if config.show_axes:
        try:
            display.display_triedron()
        except Exception:
            pass

    # Custom camera preset: 3/4 front-below view that reveals both pier groups
    # and the full substructure without the deck obscuring the far pier.
    
    display.View_Iso()              # fall back to standard iso if API differs

    display.FitAll()
    start()


# ---------------------------------------------------------------------------
# SECTION 8 — CLI ENTRY POINT
# ---------------------------------------------------------------------------

def _parse_args():
    parser = argparse.ArgumentParser(
        description="ParaBridge — parametric steel girder bridge model",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # ── Geometry ────────────────────────────────────────────────────────────
    parser.add_argument("--span",           type=float, help="Span length in chosen units")
    parser.add_argument("--units",          type=str,   default="mm", choices=["mm", "m"],
                        help="Unit system")
    parser.add_argument("--n-girders",      type=int,   dest="n_girders",
                        help="Number of main girders (≥ 3)")
    parser.add_argument("--girder-spacing", type=float, dest="girder_centroid_spacing",
                        help="Centre-to-centre girder spacing")
    parser.add_argument("--deck-thickness", type=float, dest="deck_thickness",
                        help="Deck slab thickness")
    parser.add_argument("--deck-overhang",  type=float, dest="girder_offset_from_edge",
                        help="Deck overhang beyond outermost girder")

    # ── Girder section ──────────────────────────────────────────────────────
    parser.add_argument("--girder-d",  type=float, dest="girder_section_d",  help="Girder depth")
    parser.add_argument("--girder-bf", type=float, dest="girder_section_bf_bot",
                        help="Girder flange width (both flanges)")
    parser.add_argument("--girder-tf", type=float, dest="girder_section_tf_bot",
                        help="Girder flange thickness (both flanges)")
    parser.add_argument("--girder-tw", type=float, dest="girder_section_tw", help="Girder web thickness")

    # ── Pier & pile ─────────────────────────────────────────────────────────
    parser.add_argument("--pier-height",   type=float, dest="pier_height",   help="Pier column height")
    parser.add_argument("--pier-diameter", type=float, dest="pier_diameter", help="Pier column diameter")
    parser.add_argument("--pile-length",   type=float, dest="pile_length",   help="Pile length")
    parser.add_argument("--pile-diameter", type=float, dest="pile_diameter", help="Pile diameter")
    parser.add_argument("--n-piles",       type=int,   dest="n_piles_per_cap",
                        help="Piles per pile cap")

    # ── Cross-frames ────────────────────────────────────────────────────────
    parser.add_argument("--n-crossframes", type=int, dest="n_crossframes",
                        help="Number of cross-frame planes along span")

    # ── Reinforcement ───────────────────────────────────────────────────────
    parser.add_argument("--no-rebar",       action="store_true",
                        help="Hide all reinforcement bars")
    parser.add_argument("--rebar-spacing",  type=float, dest="rebar_spacing_longitudinal",
                        help="Longitudinal rebar spacing")
    parser.add_argument("--concrete-opacity", type=float, dest="concrete_opacity",
                        help="Concrete transparency (0=opaque, 1=invisible)")

    # ── Export ──────────────────────────────────────────────────────────────
    parser.add_argument("--no-step",   action="store_true", help="Skip STEP export")
    parser.add_argument("--save-brep", action="store_true", help="Also export BREP file")
    parser.add_argument("--step-file", type=str, dest="step_filename",
                        help="STEP output path")

    # ── Display ─────────────────────────────────────────────────────────────
    parser.add_argument("--show-axes",  action="store_true", dest="show_axes",
                        help="Show XYZ axis triedron in viewer")
    parser.add_argument("--no-viewer",  action="store_true",
                        help="Skip launching the 3D viewer (export only)")
    parser.add_argument("--lod",        action="store_true", dest="pile_lod_enabled",
                        help="Use lightweight edge rings for pile stirrups (faster export)")

    return parser.parse_args()


def main():
    args = _parse_args()

    # Build BridgeConfig kwargs from whatever was explicitly passed
    kwargs = {"units": args.units}

    simple_fields = [
        "n_girders", "girder_centroid_spacing", "deck_thickness",
        "girder_offset_from_edge", "girder_section_d", "girder_section_bf_bot",
        "girder_section_tf_bot", "girder_section_tw",
        "pier_height", "pier_diameter", "pile_length", "pile_diameter",
        "n_piles_per_cap", "n_crossframes", "rebar_spacing_longitudinal",
        "concrete_opacity", "step_filename", "pile_lod_enabled",
    ]
    for field in simple_fields:
        val = getattr(args, field, None)
        if val is not None:
            kwargs[field] = val

    # All dimensional CLI args need pre-scaling to mm
    unit_scale = 1000.0 if args.units == "m" else 1.0

    # Dimensional fields passed via simple_fields also need scaling
    dimensional_simple = [
        "girder_centroid_spacing", "deck_thickness", "girder_offset_from_edge",
        "girder_section_d", "girder_section_bf_bot", "girder_section_tf_bot",
        "girder_section_tw", "rebar_spacing_longitudinal",
    ]
    for f in dimensional_simple:
        if f in kwargs:
            kwargs[f] = kwargs[f] * unit_scale

    if args.span:
        kwargs["span_length_L"] = args.span * unit_scale
    if args.pier_height:
        kwargs["pier_height"] = args.pier_height * unit_scale
    if args.pile_length:
        kwargs["pile_length"] = args.pile_length * unit_scale
    if args.pile_diameter:
        kwargs["pile_diameter"] = args.pile_diameter * unit_scale
    if args.pier_diameter:
        kwargs["pier_diameter"] = args.pier_diameter * unit_scale
    if args.no_rebar:
        kwargs["rebar_visible"] = False
    if args.no_step:
        kwargs["save_step"] = False
    if args.save_brep:
        kwargs["save_brep"] = True
    if args.show_axes:
        kwargs["show_axes"] = True
    if args.pile_lod_enabled:
        kwargs["pile_lod_enabled"] = True

    # Sync symmetric flange dims if user passed a single value
    if args.girder_section_bf_bot:
        kwargs["girder_section_bf_top"] = args.girder_section_bf_bot
    if args.girder_section_tf_bot:
        kwargs["girder_section_tf_top"] = args.girder_section_tf_bot

    config   = BridgeConfig(**kwargs)
    compound, parts = assemble_bridge(config)

    if config.save_step:
        export_step(compound, config.step_filename)
    if config.save_brep:
        export_brep(compound, config.brep_filename)
    if not args.no_viewer:
        launch_viewer(config, parts)


if __name__ == "__main__":
    main()