"""
Convert AP242 STEP files to AP203/AP214 format that pythonocc can handle better
"""

def convert_step_ap242_to_ap203(input_file, output_file):
    """
    Convert AP242 STEP format to AP203 format
    AP203 is better supported by pythonocc-core
    """
    print(f"Converting {input_file} to AP203 format...")
    
    with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    # Replace AP242 schema with AP203
    content = content.replace("'AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_WITH_LE", "'AUTOMOTIVE_DESIGN")
    content = content.replace("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_WITH_LE", "AUTOMOTIVE_DESIGN")
    content = content.replace("{ 1 0 10303 442 1 1 4 }", "{ 1 0 10303 214 1 1 1 1 }")
    
    # Also handle 3DEXPERIENCE descriptions
    content = content.replace("'3DEXPERIENCE STEP'", "'Open CASCADE Model'")
    content = content.replace("'CAx-IF Rec.Pracs.--- Model Styling and Organization---1.5--- 2016-08-15'", "'2;1'")
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)
    
    print(f"✓ Converted file saved to: {output_file}")
    print("You can now analyze this file with step_analyzer_occ.py")
    return output_file

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python convert_step_format.py <input_step_file> [output_step_file]")
        print("\nThis converts AP242/3DEXPERIENCE STEP files to AP203 format")
        print("Example: python convert_step_format.py 'L bend wire.stp' 'L_bend_wire_converted.step'")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.replace('.stp', '_converted.step').replace('.step', '_converted.step')
    
    convert_step_ap242_to_ap203(input_file, output_file)
