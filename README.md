# Fast-Quadric-Mesh-Simplification-OO

The original project is [Fast-Quadric-Mesh-Simplification](https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification) by [Spacerat](https://github.com/sp4cerat).
It is Procedure-Oriented programmed with C/C++.

For some kinds of use, I re-arrange [Fast-Quadric-Mesh-Simplification](https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification) with OO programming in C++.

Note that:
1. I do not change the main structure of original codes in [Fast-Quadric-Mesh-Simplification](https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification).
2. I create a class to wrap the original C codes in [Fast-Quadric-Mesh-Simplification](https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification).
3. I tried to change type 'double' to type 'float', but got wrong result (some coordinates of vertex are NaN). It might because of the low precision of 'float' 

What's new in Fast-Quadric-Mesh-Simplification-OO?
1. I notice that when there are vertices with same position in the vertex list, the simplified result will be wrong. 
For example, a corner of a cube may have more than 3 vertices. They have same position but different normal, uv and index.
Actually, In many representations of 3D model (file or runtime), this kind of situation occurs because position, normal, uv and index are treated as attributes of a vertex.
However, they need to be treated as one single vertex in [Fast-Quadric-Mesh-Simplification](https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification).
So, I implement a function called 'OptimizeVertices' to do merge and remove the duplicates. Call this function before the simplification and after initilization of data.

License: MIT license
