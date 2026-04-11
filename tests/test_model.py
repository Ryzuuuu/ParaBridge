"""
Unit tests for bridge_model.py
Run with:  pytest tests/
"""

import pytest
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from OCC.Core.TopoDS import TopoDS_Compound
import bridge_model as bm


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def config():
    """Default BridgeConfig instance — mirrors the spec example dimensions."""
    return bm.BridgeConfig()


@pytest.fixture
def slim_config():
    """Minimal config for fast geometry tests (fewer bars / shorter span)."""
    return bm.BridgeConfig(
        span_length_L=12000.0,
        n_girders=3,
        n_crossframes=3,
        rebar_spacing_longitudinal=500.0,
        rebar_spacing_transverse=500.0,
        pile_lod_enabled=True,      # edge proxies → faster
    )


# ---------------------------------------------------------------------------
# BridgeConfig parameter tests
# ---------------------------------------------------------------------------

class TestBridgeConfig:
    def test_derived_deck_width(self, config):
        expected = (config.n_girders - 1) * config.girder_centroid_spacing \
                   + 2.0 * config.girder_offset_from_edge
        assert abs(config.deck_width - expected) < 1e-6

    def test_girder_length_equals_span(self, config):
        assert abs(config.girder_length - config.span_length_L) < 1e-6

    def test_unit_scale_mm(self, config):
        assert config.unit_scale == 1.0

    def test_unit_scale_m(self):
        cfg = bm.BridgeConfig(units="m", span_length_L=12.0)
        assert cfg.unit_scale == 0.001
        # After scaling, span should be in mm internally
        assert abs(cfg.span_length_L - 12.0) < 1e-6   # already scaled by post_init

    def test_custom_span(self):
        cfg = bm.BridgeConfig(span_length_L=15000.0)
        assert abs(cfg.span_length_L - 15000.0) < 1e-6

    def test_all_required_params_present(self, config):
        required = [
            "units", "span_length_L", "n_girders", "girder_centroid_spacing",
            "girder_offset_from_edge", "n_crossframes", "pier_location_x",
            "n_lanes", "lane_width",
            "girder_section_d", "girder_section_bf_bot", "girder_section_bf_top",
            "girder_section_tf_bot", "girder_section_tf_top", "girder_section_tw",
            "girder_material",
            "deck_thickness", "deck_slab_segment_length", "deck_cover",
            "deck_material_opacity",
            "pier_diameter", "pier_height",
            "pier_cap_length", "pier_cap_top_width", "pier_cap_bottom_width",
            "pier_cap_depth", "pier_cap_elevation",
            "n_piles_per_cap", "pile_diameter", "pile_length", "pile_spacing",
            "pile_cap_length", "pile_cap_width", "pile_cap_depth", "pile_cap_elevation",
            "rebar_main_diameter", "rebar_transverse_diameter",
            "rebar_spacing_longitudinal", "rebar_spacing_transverse",
            "rebar_cover", "rebar_visible", "concrete_opacity",
            "show_axes", "background_color",
            "save_step", "step_filename",
            "save_brep", "brep_filename",
            "render_size",
        ]
        for param in required:
            assert hasattr(config, param), f"Missing BridgeConfig parameter: {param}"


# ---------------------------------------------------------------------------
# Component factory tests
# ---------------------------------------------------------------------------

class TestFactories:
    def test_i_section_returns_shape(self, config):
        shape = bm.create_i_section(
            config.girder_section_d,
            config.girder_section_bf_bot,
            config.girder_section_bf_top,
            config.girder_section_tf_bot,
            config.girder_section_tf_top,
            config.girder_section_tw,
            config.girder_length,
        )
        assert shape is not None
        assert hasattr(shape, "ShapeType")

    def test_rectangular_prism_returns_shape(self, config):
        shape = bm.create_rectangular_prism(1000.0, 200.0, 5000.0)
        assert shape is not None

    def test_circular_pier_returns_shape(self, config):
        shape = bm.create_circular_pier(config.pier_diameter, config.pier_height)
        assert shape is not None

    def test_trapezoidal_cap_returns_shape(self, config):
        shape = bm.create_trapezoidal_pier_cap(
            config.pier_cap_length,
            config.pier_cap_top_width,
            config.pier_cap_bottom_width,
            config.pier_cap_depth,
        )
        assert shape is not None

    def test_pile_returns_shape(self, config):
        shape = bm.create_pile(config.pile_diameter, config.pile_length)
        assert shape is not None

    def test_pile_cap_returns_shape(self, config):
        shape = bm.create_pile_cap(
            config.pile_cap_length, config.pile_cap_width, config.pile_cap_depth
        )
        assert shape is not None

    def test_deck_rebar_grid_returns_bars(self, slim_config):
        bars = bm.create_rebar_grid_for_deck(
            slim_config.deck_width,
            slim_config.span_length_L,
            slim_config.deck_cover,
            slim_config.rebar_main_diameter,
            slim_config.rebar_spacing_longitudinal,
            slim_config.rebar_spacing_transverse,
            slim_config.deck_thickness,
        )
        assert isinstance(bars, list)
        assert len(bars) > 0

    def test_circular_section_rebar_count(self, config):
        bars = bm.create_rebar_for_circular_section(
            config.pier_n_long_bars,
            config.pier_diameter / 2.0,
            config.rebar_main_diameter,
            config.rebar_cover,
            config.pier_height,
        )
        assert len(bars) == config.pier_n_long_bars

    def test_rect_section_rebar_returns_bars(self, slim_config):
        bars = bm.create_rebar_for_rect_section(
            slim_config.pier_cap_length,
            slim_config.pier_cap_bottom_width,
            slim_config.pier_cap_depth,
            slim_config.rebar_main_diameter,
            slim_config.rebar_cover,
            slim_config.rebar_spacing_longitudinal,
        )
        assert isinstance(bars, list)
        assert len(bars) > 0

    def test_pile_stirrups_proxy_mode(self, slim_config):
        rings = bm.create_pile_stirrups(
            0.0, 0.0, -5000.0,
            slim_config.pile_length,
            slim_config.pile_diameter,
            slim_config.pile_rebar_diameter,
            slim_config.pile_rebar_spacing,
            use_proxy=True,
        )
        assert isinstance(rings, list)
        assert len(rings) > 0


# ---------------------------------------------------------------------------
# Assembly tests
# ---------------------------------------------------------------------------

class TestAssembly:
    def test_bridge_assembly_returns_compound(self, slim_config):
        compound, parts = bm.assemble_bridge(slim_config)
        assert isinstance(compound, TopoDS_Compound)
        assert not compound.IsNull()

    def test_all_part_keys_present(self, slim_config):
        _, parts = bm.assemble_bridge(slim_config)
        expected_keys = {
            "girders", "crossframes", "deck",
            "pier_caps", "piers", "pile_caps", "piles", "pile_stirrups",
            "deck_rebars", "pier_rebars", "pier_cap_rebars", "pile_cap_rebars",
        }
        assert expected_keys.issubset(set(parts.keys()))

    def test_girder_count(self, slim_config):
        _, parts = bm.assemble_bridge(slim_config)
        assert len(parts["girders"]) == slim_config.n_girders

    def test_deck_segment_count(self, slim_config):
        import math
        expected = max(1, math.ceil(
            slim_config.span_length_L / slim_config.deck_slab_segment_length
        ))
        _, parts = bm.assemble_bridge(slim_config)
        assert len(parts["deck"]) == expected

    def test_pier_count(self, slim_config):
        _, parts = bm.assemble_bridge(slim_config)
        assert len(parts["piers"])     == 2
        assert len(parts["pier_caps"]) == 2
        assert len(parts["pile_caps"]) == 2

    def test_pile_count(self, slim_config):
        _, parts = bm.assemble_bridge(slim_config)
        # 4 piles per pier group, 2 pier groups → 8 piles total
        assert len(parts["piles"]) == 4 * 2

    def test_crossframe_count(self, slim_config):
        _, parts = bm.assemble_bridge(slim_config)
        # 2 diagonals × (n_girders-1) bays × n_crossframes planes
        expected = 2 * (slim_config.n_girders - 1) * slim_config.n_crossframes
        assert len(parts["crossframes"]) == expected

    def test_rebar_present_when_visible(self, slim_config):
        _, parts = bm.assemble_bridge(slim_config)
        assert len(parts["deck_rebars"])     > 0
        assert len(parts["pier_rebars"])     > 0
        assert len(parts["pier_cap_rebars"]) > 0
        assert len(parts["pile_cap_rebars"]) > 0

    def test_rebar_absent_when_hidden(self):
        cfg = bm.BridgeConfig(rebar_visible=False)
        _, parts = bm.assemble_bridge(cfg)
        assert len(parts["deck_rebars"])     == 0
        assert len(parts["pier_rebars"])     == 0
        assert len(parts["pier_cap_rebars"]) == 0
        assert len(parts["pile_cap_rebars"]) == 0

    def test_parametric_span(self):
        cfg = bm.BridgeConfig(span_length_L=15000.0)
        _, parts = bm.assemble_bridge(cfg)
        assert len(parts["girders"]) == cfg.n_girders