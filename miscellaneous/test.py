from xml.etree import ElementTree as ET

# Parse your URDF
tree = ET.parse('/home/ros/eb65_ws/src/eb65_description/urdf/eb65.urdf')
root = tree.getroot()

# Default inertia for small components (adjust as needed)
DEFAULT_INERTIA = {
    'mass': '0.1',
    'ixx': '0.0001', 'ixy': '0', 'ixz': '0',
    'iyy': '0.0001', 'iyz': '0', 'izz': '0.0001'
}

# For larger base components
BASE_INERTIA = {
    'mass': '1.0',
    'ixx': '0.01', 'ixy': '0', 'ixz': '0',
    'iyy': '0.01', 'iyz': '0', 'izz': '0.01'
}

# Add inertial tags to all links without them
for link in root.findall('link'):
    if link.find('inertial') is None:
        inertial = ET.SubElement(link, 'inertial')
        origin = ET.SubElement(inertial, 'origin')
        origin.set('xyz', '0 0 0')
        origin.set('rpy', '0 0 0')
        
        mass = ET.SubElement(inertial, 'mass')
        inertia = ET.SubElement(inertial, 'inertia')
        
        # Use different inertia for base links
        if 'base' in link.get('name').lower():
            mass.set('value', BASE_INERTIA['mass'])
            inertia.set('ixx', BASE_INERTIA['ixx'])
            inertia.set('ixy', BASE_INERTIA['ixy'])
            inertia.set('ixz', BASE_INERTIA['ixz'])
            inertia.set('iyy', BASE_INERTIA['iyy'])
            inertia.set('iyz', BASE_INERTIA['iyz'])
            inertia.set('izz', BASE_INERTIA['izz'])
        else:
            mass.set('value', DEFAULT_INERTIA['mass'])
            inertia.set('ixx', DEFAULT_INERTIA['ixx'])
            inertia.set('ixy', DEFAULT_INERTIA['ixy'])
            inertia.set('ixz', DEFAULT_INERTIA['ixz'])
            inertia.set('iyy', DEFAULT_INERTIA['iyy'])
            inertia.set('iyz', DEFAULT_INERTIA['iyz'])
            inertia.set('izz', DEFAULT_INERTIA['izz'])
        
        print(f"Added inertial tag to {link.get('name')}")

# Save the modified URDF
tree.write('/home/ros/eb65_ws/src/eb65_description/urdf/eb65_with_inertia.urdf', 
           encoding='utf-8', xml_declaration=True)
print("Saved to eb65_with_inertia.urdf")