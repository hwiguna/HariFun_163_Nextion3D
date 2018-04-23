class Camera
{
  public:
    Point3D pos;
    Point3D rot;

    Camera(Point3D p, Point3D r) {
      pos = p;
      rot = r;
    };

    void Update(int stp, char key)
    {
            if (key=='q') pos.y -= stp;
            if (key=='e') pos.y += stp;
      
      //      if (key=='w') pos.z += stp;
      //      if (key=='s') pos.z -= stp;
      //
      //      if (key=='a') pos.x -= stp;
      //      if (key=='d') pos.x += stp;

      float x = sin(rot.y) * stp;
      float y = cos(rot.y) * stp;

      if (key == 'w') {pos.x += x;pos.z += y;}
      if (key == 's') {pos.x -= x;pos.z -= y;}
      if (key == 'a') {pos.x -= y;pos.z += x;}
      if (key == 'd') {pos.x += y;pos.z -= x;}

    }
};
