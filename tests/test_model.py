"""
Unit tests for bridge_model.py
"""
import pytest
from OCC.Core.TopoDS import TopoDS_Compound
import bridge_model as bm

def test_bridge_assembly():
    comp, parts = bm.assemble_bridge()
    
    # Check that it returns a valid pythonOCC TopoDS_Compound
    assert isinstance(comp, TopoDS_Compound)
    assert not comp.IsNull()
    
    # Verify all expected components are generated according to FOSSEE specs
    assert "girders" in parts
    assert len(parts["girders"]) == bm.n_girders
    
    assert "deck" in parts
    assert len(parts["deck"]) == 1
    
    assert "piers" in parts
    assert len(parts["piers"]) == 2
    
    assert "pier_caps" in parts
    assert len(parts["pier_caps"]) == 2
    
    assert "pile_caps" in parts
    assert len(parts["pile_caps"]) == 2
    
    assert "piles" in parts
    assert len(parts["piles"]) == bm.n_piles_per_cap * 2
    
    assert "crossframes" in parts
    assert len(parts["crossframes"]) > 0

def test_parametric_span():
    # Save the original
    original_span = bm.span_length_L
    
    try:
        # Change the parametric variable
        bm.span_length_L = 15000.0
        girders = bm.build_girders()
        assert len(girders) == bm.n_girders
    finally:
        # Restore
        bm.span_length_L = original_span
