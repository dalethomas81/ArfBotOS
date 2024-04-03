# Tests CoDeSys native import/export functionality.
from __future__ import print_function
proj = projects.primary
# We're interested in POU nodes:
POUGuid = Guid("6f9dac99-8de1-4efc-8465-68ac443b7d08")
# We collect all POU nodes in that list.
pous = []
# From the parent node on, we recursively add POU nodes:
def CollectPous(node):
    if node.type == POUGuid:
        pous.append(node)
    else:
        for child in node.get_children():
            CollectPous(child)
# Now we collect all the leaf nodes.
for node in proj.get_children():
    CollectPous(node)
# We print everything just to know what's going on.
for i in pous:
    print("found: ", i.type, i.guid, i.get_name())
# And now we export the files.
for candidate in pous:
    # We create a list of objects to export:
    # The object itsself
    objects = [candidate]
    
    # And sub-objects (POUs can have actions, properties, ...)
    objects.extend(candidate.get_children())
    
    # And the parent folders.
    parent = candidate.parent
    while ((not parent.is_root) and parent.is_folder):
        objects.append(parent)
        parent = parent.parent
    
    # Create an unique file name:
    filename = "c:\\test\\%s__%s.export" % (candidate.get_name(), candidate.guid)
    
    # print some user information
    print("exporting ", len(objects), " objects to: ", filename)
    
    # and actually export the project.
    proj.export_native(objects, filename)
print ("script finished.")