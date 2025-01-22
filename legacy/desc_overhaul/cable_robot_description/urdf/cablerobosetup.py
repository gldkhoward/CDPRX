#!/usr/bin/env python3

from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

class CDPRGenerator:
    def __init__(self):
        # Configuration parameters
        self.platform_size = [0.4, 0.4, 0.05]  # x, y, z in meters
        self.platform_mass = 1.0  # kg
        self.pillar_height = 3.0  # meters
        self.pillar_radius = 0.05  # meters
        self.pillar_positions = [
            [1.5, 1.5],   # Front right
            [-1.5, 1.5],  # Front left
            [-1.5, -1.5], # Back left
            [1.5, -1.5]   # Back right
        ]
        self.cable_radius = 0.005  # meters
        self.cable_attachment_points = [
            [0.2, 0.2],   # Front right
            [-0.2, 0.2],  # Front left
            [-0.2, -0.2], # Back left
            [0.2, -0.2]   # Back right
        ]

    def create_inertial_element(self, mass, ixx, iyy, izz):
        inertial = Element('inertial')
        mass_elem = SubElement(inertial, 'mass')
        mass_elem.text = str(mass)
        
        inertia = SubElement(inertial, 'inertia')
        for name, value in [('ixx', ixx), ('iyy', iyy), ('izz', izz),
                          ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
            elem = SubElement(inertia, name)
            elem.text = str(value)
        
        return inertial

    def create_material_element(self, r, g, b, a=1):
        material = Element('material')
        for prop in ['ambient', 'diffuse']:
            elem = SubElement(material, prop)
            elem.text = f"{r} {g} {b} {a}"
        specular = SubElement(material, 'specular')
        specular.text = "0.1 0.1 0.1 1"
        return material

    def create_platform(self):
        link = Element('link')
        link.set('name', 'platform')
        
        pose = SubElement(link, 'pose')
        pose.text = '0 0 2 0 0 0'
        
        # Calculate platform inertia (box)
        x, y, z = self.platform_size
        mass = self.platform_mass
        ixx = (1/12) * mass * (y*y + z*z)
        iyy = (1/12) * mass * (x*x + z*z)
        izz = (1/12) * mass * (x*x + y*y)
        
        link.append(self.create_inertial_element(mass, ixx, iyy, izz))
        
        for name in ['collision', 'visual']:
            elem = SubElement(link, name)
            elem.set('name', f'platform_{name}')
            geom = SubElement(elem, 'geometry')
            box = SubElement(geom, 'box')
            size = SubElement(box, 'size')
            size.text = f"{x} {y} {z}"
            
            if name == 'visual':
                elem.append(self.create_material_element(0, 0.5, 0))
        
        return link

    def create_pillar(self, index, x, y):
        link = Element('link')
        link.set('name', f'pillar_{index+1}')
        
        pose = SubElement(link, 'pose')
        pose.text = f"{x} {y} {self.pillar_height/2} 0 0 0"
        
        # Using large mass/inertia for static behavior
        link.append(self.create_inertial_element(1e6, 1e6, 1e6, 1e6))
        
        for name in ['collision', 'visual']:
            elem = SubElement(link, name)
            elem.set('name', f'pillar_{index+1}_{name}')
            geom = SubElement(elem, 'geometry')
            cyl = SubElement(geom, 'cylinder')
            radius = SubElement(cyl, 'radius')
            radius.text = str(self.pillar_radius)
            length = SubElement(cyl, 'length')
            length.text = str(self.pillar_height)
            
            if name == 'visual':
                elem.append(self.create_material_element(0.5, 0.5, 0.5))
        
        return link

    def create_cable(self, index, attach_x, attach_y):
        link = Element('link')
        link.set('name', f'cable_{index+1}')
        
        pose = SubElement(link, 'pose')
        pose.text = f"relative_to=platform {attach_x} {attach_y} 0 0 0 0"
        
        # Minimal mass/inertia for cables
        link.append(self.create_inertial_element(1e-6, 1e-6, 1e-6, 1e-6))
        
        visual = SubElement(link, 'visual')
        visual.set('name', f'cable_{index+1}_visual')
        geom = SubElement(visual, 'geometry')
        cyl = SubElement(geom, 'cylinder')
        radius = SubElement(cyl, 'radius')
        radius.text = str(self.cable_radius)
        length = SubElement(cyl, 'length')
        length.text = '1.0'  # Default length, will be adjusted by prismatic joint
        
        visual.append(self.create_material_element(0, 0, 0))  # Black cables
        
        return link

    def create_cable_joint(self, index):
        joint = Element('joint')
        joint.set('name', f'cable_{index+1}_prismatic')
        joint.set('type', 'prismatic')
        
        parent = SubElement(joint, 'parent')
        parent.text = 'platform'
        child = SubElement(joint, 'child')
        child.text = f'cable_{index+1}'
        
        axis = SubElement(joint, 'axis')
        xyz = SubElement(axis, 'xyz')
        xyz.text = '0 0 1'
        
        limit = SubElement(axis, 'limit')
        for name, value in [('lower', 0), ('upper', 5.0),
                          ('effort', 1000), ('velocity', 1.0)]:
            elem = SubElement(limit, name)
            elem.text = str(value)
        
        dynamics = SubElement(axis, 'dynamics')
        damping = SubElement(dynamics, 'damping')
        damping.text = '1.0'
        spring_ref = SubElement(dynamics, 'spring_reference')
        spring_ref.text = '0'
        spring_stiff = SubElement(dynamics, 'spring_stiffness')
        spring_stiff.text = '100'
        
        return joint

    def create_pillar_joint(self, index):
        joint = Element('joint')
        joint.set('name', f'pillar_{index+1}_fixed')
        joint.set('type', 'fixed')
        
        parent = SubElement(joint, 'parent')
        parent.text = 'world'
        child = SubElement(joint, 'child')
        child.text = f'pillar_{index+1}'
        
        return joint

    def create_joint_state_publisher(self):
        plugin = Element('plugin')
        plugin.set('name', 'joint_state_publisher')
        plugin.set('filename', 'libgazebo_ros_joint_state_publisher.so')
        
        ns = SubElement(plugin, 'robotNamespace')
        ns.text = 'cdpr'
        
        joint_names = SubElement(plugin, 'jointName')
        joint_names.text = ', '.join([f'cable_{i+1}_prismatic' for i in range(4)])
        
        rate = SubElement(plugin, 'updateRate')
        rate.text = '50'
        
        return plugin

    def generate_sdf(self):
        sdf = Element('sdf')
        sdf.set('version', '1.6')
        
        model = SubElement(sdf, 'model')
        model.set('name', 'cdpr')
        
        # Add platform
        model.append(self.create_platform())
        
        # Add pillars and their joints
        for i, (x, y) in enumerate(self.pillar_positions):
            model.append(self.create_pillar(i, x, y))
            model.append(self.create_pillar_joint(i))
        
        # Add cables and their joints
        for i, (x, y) in enumerate(self.cable_attachment_points):
            model.append(self.create_cable(i, x, y))
            model.append(self.create_cable_joint(i))
        
        # Add joint state publisher plugin
        model.append(self.create_joint_state_publisher())
        
        # Convert to pretty XML string
        rough_string = tostring(sdf, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")

def main():
    generator = CDPRGenerator()
    sdf_content = generator.generate_sdf()
    
    # Save to file
    with open('cdpr.sdf', 'w') as f:
        f.write(sdf_content)

if __name__ == '__main__':
    main()