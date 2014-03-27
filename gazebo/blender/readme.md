##Quick Primer on Blender

###Meshes and Textures

Since we can export COLLADA models from blender, we can use meshes and put textures on meshes in blender to build our world. A mesh is a collection of vertices. In this collection, we define edges, which are lines that connect two vertices. We also define faces, which connect 3 or more vertices.

In blender, you can assign materials or textures to a mesh by navigating to the properties window for that mesh, selecting "Add new Material", and then "Add new Texture". In the texture properties panel, select "Image" from the texture type dropdown. Then select your image. You will then need to UV map the image to your mesh, as described [here](http://wiki.blender.org/index.php/Doc:2.6/Manual/Textures/Mapping/UV/Unwrapping).

A few quick notes: Remember that a face has a normal (the side of the face that you wish to paint). Also, after mapping a texture or image to a mesh, you may notice that it still looks dark. This is likely because of the lack of light on the object. When UV mapping, always remember to select "UV" coordinates for the mapping coordinates.
