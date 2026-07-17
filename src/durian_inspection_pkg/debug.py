import sqlite3
import os

def migrate_pcd_to_db(pcd_file, db_name="durian_data.db"):
    if not os.path.exists(pcd_file):
        print(f"Error: File {pcd_file} not found!")
        return

    print(f"Opening {pcd_file} and preparing migration...")
    
    # 1. 初始化 SQLite 数据库
    conn = sqlite3.connect(db_name)
    cursor = conn.cursor()
    cursor.execute('DROP TABLE IF EXISTS point_clouds') # 如果表存在则覆盖，或者保留原有
    cursor.execute('''CREATE TABLE point_clouds 
                      (id INTEGER PRIMARY KEY, x REAL, y REAL, z REAL, rgb REAL)''')

    # 2. 读取 PCD 文件
    points = []
    with open(pcd_file, "r") as f:
        # 跳过文件头信息
        header_passed = False
        for line in f:
            if not header_passed:
                if line.strip() == "DATA ascii":
                    header_passed = True
                continue
            
            # 解析坐标点数据
            parts = line.split()
            if len(parts) >= 4:
                points.append((float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])))
    
    # 3. 批量写入数据库 (比一条条插入快几百倍)
    print(f"Migrating {len(points)} points to database...")
    cursor.executemany("INSERT INTO point_clouds (x, y, z, rgb) VALUES (?, ?, ?, ?)", points)
    
    conn.commit()
    conn.close()
    print(f"Migration completed! Data saved in {db_name}")

if __name__ == '__main__':
    # 修改为你实际的路径
    pcd_path = "/home/johny/durian_ws/durian_tree.pcd"
    migrate_pcd_to_db(pcd_path)