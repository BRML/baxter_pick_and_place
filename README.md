# baxter-pick-and-place
Pick &amp; place demonstration with the baxter research robot in collaboration 
with DFKI Saarbruecken.

## Description of demonstration
An eye tracker is used to select and trigger an object lying on a table in 
front of the baxter robot. A network service is used to classify the selected 
object and returns 5 object labels and their respective probabilities of being 
the object being asked for. 

The baxter robot looks for objects on the table, calls the network service on 
them in turn, selects the most probable object, picks it up and places it in a 
pre-defined location.
