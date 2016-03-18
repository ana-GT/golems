import bpy
import subprocess
import os
import sys

from bpy.types import(Operator)

######################################
print("Convert .ply to .wrl")

# Start by erasing cube
bpy.ops.object.select_all( action='DESELECT' )
mScene = bpy.context.scene
# Erase
for ob in mScene.objects:
  ob.select = ob.type == 'MESH'
bpy.ops.object.delete()

print('Should have erased cube')

# Read .ply file
argv = sys.argv
argv = argv[argv.index("--") + 1:]
filename = argv[0]

print("[DEBUG] Filename :  " + filename )
mesh_name = os.path.basename( os.path.splitext(filename)[0] )
print( " Mesh name: " + mesh_name )



# Import .ply file
bpy.ops.import_mesh.ply(filepath=filename)

# It is in meters. Scale it to centimeters for damn Graspit
for obi in bpy.data.objects:
  if obi.type == 'MESH':
    obi.scale=((1000,1000,1000))

# Export wrl
wrl_filename = mesh_name + '.wrl'
bpy.ops.export_scene.vrml2(filepath=wrl_filename, axis_up='Z', axis_forward='Y')
