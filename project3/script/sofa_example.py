# to be able to create SOFA objects you need to first load the plugins that implement them.
# For simplicity you can load the plugin "SofaComponentAll" that will load all most
# common sofa objects.
import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

# to create elements like Node or objects
import Sofa.Core

def createScene(rootNode):
        #Doesn't do anything yet
    rootNode.addObject("OglGrid", nbSubdiv=10, size=1000)
    confignode = rootNode.addChild("Config")
    confignode.addObject('RequiredPlugin', name="SofaPython3", printLog=False)
    confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    #Creating the sphere
    sphere = rootNode.addChild("sphere")
    sphere.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=50)

    #### visualization
    sphereVisu = sphere.addChild("VisualModel")
    sphereVisu.loader = sphereVisu.addObject('MeshObjLoader', name="loader", filename="mesh/ball.obj")
    sphereVisu.addObject('OglModel', name="model", src="@loader", scale3d=[50]*3, color=[0., 1., 0.], updateNormals=False)
    sphereVisu.addObject('RigidMapping')
