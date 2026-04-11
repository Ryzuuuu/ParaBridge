from OCC.Core.gp import gp_Vec, gp_Trsf
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox

from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.BRep import BRep_Builder
from OCC.Display.SimpleGui import init_display


def create_i_section(length, top_width, bot_width, depth, top_flange_thickness, bot_flange_thickness, web_thickness):
    """
    Create an asymmetric I-section CAD model (per IS 800) with distinct top and bottom flanges.

    Parameters:
    - length: Length of the I-section (along extrusion axis)
    - top_width: Width of the top flange (compression flange for simply supported)
    - bot_width: Width of the bottom flange (tension flange for simply supported)
    - depth: Total depth of the I-section (vertical dimension)
    - top_flange_thickness: Thickness of the top flange
    - bot_flange_thickness: Thickness of the bottom flange
    - web_thickness: Thickness of the web (centered)

    Returns:
    - i_section_solid: The asymmetric I-section CAD model as a TopoDS_Solid
    
    Engineering note: Asymmetric sections are used in composite bridges to optimize
    interaction between steel girder and RC deck slab.
    """
    # Compute web height between flanges
    web_height = depth - top_flange_thickness - bot_flange_thickness

    # Create the bottom flange (centered at Y=0)
    bottom_flange = BRepPrimAPI_MakeBox(length, bot_width, bot_flange_thickness).Shape()
    trsf_bot = gp_Trsf()
    trsf_bot.SetTranslation(gp_Vec(0, -bot_width / 2.0, 0))
    bottom_flange = BRepBuilderAPI_Transform(bottom_flange, trsf_bot, True).Shape()

    # Create the top flange at the correct elevation (centered at Y=0)
    top_flange = BRepPrimAPI_MakeBox(length, top_width, top_flange_thickness).Shape()
    trsf_top = gp_Trsf()
    trsf_top.SetTranslation(gp_Vec(0, -top_width / 2.0, depth - top_flange_thickness))
    top_flange = BRepBuilderAPI_Transform(top_flange, trsf_top, True).Shape()

    # Create the web (centered at Y=0, between flanges)
    web = BRepPrimAPI_MakeBox(length, web_thickness, web_height).Shape()
    trsf_web = gp_Trsf()
    trsf_web.SetTranslation(gp_Vec(0, -web_thickness / 2.0, bot_flange_thickness))
    web = BRepBuilderAPI_Transform(web, trsf_web, True).Shape()

    # Combine the flanges and web using a Compound for O(1) assembly
    
    builder = BRep_Builder()
    i_section_compound = TopoDS_Compound()
    builder.MakeCompound(i_section_compound)
    
    builder.Add(i_section_compound, bottom_flange)
    builder.Add(i_section_compound, top_flange)
    builder.Add(i_section_compound, web)

    return i_section_compound


if __name__ == "__main__":
    length = 1000.0
    top_width = 100.0  # Top flange width
    bot_width = 120.0  # Bottom flange width (slightly wider for tension)
    depth = 200.0      # Total depth
    top_flange_thickness = 10.0
    bot_flange_thickness = 12.0
    web_thickness = 5.0

    i_section = create_i_section(length, top_width, bot_width, depth, 
                                  top_flange_thickness, bot_flange_thickness, web_thickness)

    # Visualization
    display, start_display, add_menu, add_function_to_menu = init_display()

    # Show the I-section model
    display.DisplayShape(i_section, update=True)
    display.FitAll()
    start_display()