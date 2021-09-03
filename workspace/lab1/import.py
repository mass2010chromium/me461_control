from __future__ import print_function
import os
import sys
import subprocess

FPU_LIB_FILE="C2000Ware_3_02_00_00_F28379D/libraries/dsp/FPU/c28/lib/c28x_fpu_dsp_library.lib"
DRIVER_LIB_FILE="C2000Ware_3_02_00_00_F28379D/driverlib/f2837xd/driverlib/ccs/Debug/driverlib.lib"

COMMON_PATH="C2000Ware_3_02_00_00_F28379D/device_support/f2837xd/common/include"
DLIB_PATH="C2000Ware_3_02_00_00_F28379D/driverlib/f2837xd/driverlib"
DSPLIB_ROOT="C2000Ware_3_02_00_00_F28379D/libraries/dsp/FPU/c28/include"
C2000WARE_HEADERS="C2000Ware_3_02_00_00_F28379D/device_support/f2837xd/headers/include"

names_map = {
        "C2000WARE_COMMON_INCLUDE": COMMON_PATH,
        "C2000WARE_DLIB_ROOT": DLIB_PATH,
        "C2000WARE_DSPLIB_ROOT": DSPLIB_ROOT,
        "C2000WARE_HEADERS_INCLUDE": C2000WARE_HEADERS
    }

LABSTARTER_LOC_WIN="\\C2000Ware_3_02_00_00_F28379D\\device_support\\f2837xd\\examples\\cpu1\\labstarter\\cpu01\\ccs\\labstarter.projectspec"
LABSTARTER_LOC_UNIX="/C2000Ware_3_02_00_00_F28379D/device_support/f2837xd/examples/cpu1/labstarter/cpu01/ccs/labstarter.projectspec"

LABSTARTER_LOC = LABSTARTER_LOC_WIN

repo_root = subprocess.getoutput("git rev-parse --show-toplevel")
#repo_root = "test"

import xml.etree.ElementTree as ET
project_file = ET.parse("_project")
root = project_file.getroot()
for child in root:
    if child.tag == "linkedResources":
        for link in child:
            if link.find("name").text == "c28x_fpu_dsp_library.lib":
                link.find("location").text = repo_root + "/" + FPU_LIB_FILE
            elif link.find("name").text == "driverlib.lib":
                link.find("location").text = repo_root + "/" + DRIVER_LIB_FILE
    if child.tag == "variableList":
        for var in child:
            name = var.find("name").text
            if name in names_map:
                var.find("value").text = repo_root + "/" + names_map[name];
            
project_file.write(".project", encoding='UTF-8', xml_declaration=True)

ccs_project_file = ET.parse("_ccsproject")
ccs_out = open(".ccsproject", 'wb')
ccs_out.write(b"""<?xml version="1.0" encoding="UTF-8" ?>\n<?ccsproject version="1.0"?>\n""")
root = ccs_project_file.getroot()
for child in root:
    if child.tag == "origin":
        child.set("value", repo_root + LABSTARTER_LOC)
import io
xml_out = io.BytesIO()
ccs_project_file.write(ccs_out, encoding='UTF-8')
ccs_out.close()