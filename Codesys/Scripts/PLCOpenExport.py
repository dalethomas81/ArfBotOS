# Tests CoDeSys PLCOpen import/export functionality.
from __future__ import print_function
import os

proj = projects.primary

objects = proj.get_children()

# Create an unique file name:
dir_path = os.path.dirname(os.path.realpath(__file__))
filename = str(os.path.join(dir_path, 'ArfBot.xml'))
#filename = "c:\\test\\ArfBot.xml"

#export_xml(objects, reporter=None, path=None, recursive=False, export_folder_structure=False, declarations_as_plaintext=False)
proj.export_xml(objects, path=filename, recursive=True, export_folder_structure=True, declarations_as_plaintext=True)

print ("script finished.")