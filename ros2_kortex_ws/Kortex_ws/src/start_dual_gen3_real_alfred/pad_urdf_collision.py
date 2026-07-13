import xml.etree.ElementTree as ET

def main():
    urdf_path = "/home/alfred/workspace/ros2_kortex_ws/scripts/start_dual_gen3_real_alfred/my_dual_gen3.urdf"
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    count = 0
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            if geometry is not None:
                mesh = geometry.find('mesh')
                if mesh is not None:
                    # Set the scale to 1.15 1.15 1.15 to widen the collision volume
                    mesh.set('scale', '1.15 1.15 1.15')
                    count += 1
    
    tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
    print(f"Successfully padded {count} collision meshes with scale='1.15' in {urdf_path}")

if __name__ == '__main__':
    main()
