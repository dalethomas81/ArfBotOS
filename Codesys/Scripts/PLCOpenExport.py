# Tests CoDeSys PLCOpen import/export functionality.
from __future__ import print_function
import os

proj = projects.primary

objects = proj.get_children()

# Save next to the .project file (e.g. Codesys/ArfBot.xml)
filename = str(os.path.splitext(proj.path)[0] + '.xml')

#export_xml(objects, reporter=None, path=None, recursive=False, export_folder_structure=False, declarations_as_plaintext=False)
proj.export_xml(objects, path=filename, recursive=True, export_folder_structure=True, declarations_as_plaintext=True)

print ("script finished.")