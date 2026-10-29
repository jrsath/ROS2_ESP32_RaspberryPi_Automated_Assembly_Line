# Separate box and lid meshes extracted from the university CAD.
def package_sdf(name, lid=False):
    mesh = "lid" if lid else "box"
    return (
        f'<sdf version="1.9"><model name="{name}"><link name="{name}__part">'
        "<gravity>false</gravity><inertial><mass>0.03</mass><inertia>"
        "<ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz>"
        '</inertia></inertial><visual name="shape"><geometry><mesh>'
        f"<uri>model://packaging/{mesh}.stl</uri></mesh></geometry>"
        "<material><ambient>0.65 0.45 0.24 1</ambient>"
        "<diffuse>0.65 0.45 0.24 1</diffuse></material></visual>"
        "</link></model></sdf>"
    )
