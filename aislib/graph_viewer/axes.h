#ifndef AXES_HH
#define AXES_HH

#include <iostream>
#include <fstream>

// forward decl
class Axes;
std::ostream& operator<<(std::ostream &stream, const Axes& axes);

class Axes
{
  public:
    Axes(double len = 1.) : 
      _len(len)
    {}

    double getLen() const { return _len; }

    void saveToFile(const char* filename, bool append = false)
    {
      if (append) {
        std::ofstream fout(filename, std::ios::app);
        fout << *this;
        fout.close();
      } else {
        std::ofstream fout(filename);
        fout << *this;
        fout.close();
      }
    }

  private:
    double _len;
};

inline std::ostream& operator<<(std::ostream &stream, const Axes& axes)
{
  double len = axes.getLen();

  // top-level transform around everything
  stream << "#coordinate axes" << std::endl;
  stream << "Transform {\n";
  stream << "   translation 0 0 0\n";
  stream << "   children [\n";

  stream << "Shape {\n";
  stream << "  appearance Appearance {" << std::endl;
  stream << "    material Material { " << std::endl;
  stream << "         diffuseColor 0.8 0.8 0.8 " << std::endl;
  stream << "         ambientIntensity 0.2 " << std::endl;
  stream << "         emissiveColor 0.0 0.0 0.0 " << std::endl;
  stream << "         specularColor 0.0 0.0 0.0 " << std::endl;
  stream << "         shininess 0.2 " << std::endl;
  stream << "         transparency 0.0 " << std::endl;
  stream << "    } " << std::endl;
  stream << "  } " << std::endl;
  stream << "  geometry IndexedLineSet {\n";
  stream << "     coord Coordinate {\n";
  stream << "        point [ 0 0 0, " << len << " 0 0, 0 " << len << " 0, 0 0 " << len << "]\n";
  stream << "     }\n";
  stream << "     coordIndex [ 0 1 -1 0 2 -1 0 3 -1]\n";
  stream << "     color Color {\n"
         << "       color [0.8 0.0 0.0, 0.0 0.8 0.0, 0.0 0.0 0.8]\n"
         << "     }\n";
  stream << "     colorPerVertex FALSE\n";
  stream << "    }\n";
  stream << "}\n";

  // X Arrow
  stream << "Transform {\n";
  stream << "   rotation 0 0 1 -1.5707963\n";
  stream << "   translation " <<  len << " 0 0\n";
  stream << "   children [\n";
  stream << "   Shape {\n";
  stream << "     appearance Appearance {" << std::endl;
  stream << "       material Material { " << std::endl;
  stream << "          diffuseColor 0.8 0.0 0.0 " << std::endl;
  stream << "          ambientIntensity 0.2 " << std::endl;
  stream << "          emissiveColor 0.0 0.0 0.0 " << std::endl;
  stream << "          specularColor 0.0 0.0 0.0 " << std::endl;
  stream << "          shininess 0.2 " << std::endl;
  stream << "          transparency 0.0 " << std::endl;
  stream << "       } " << std::endl;
  stream << "     } " << std::endl;
  stream << "     geometry Cone {\n";
  stream << "        bottomRadius " << len / 100. << "\n";
  stream << "        height " << len /10. << "\n";
  stream << "        }\n";
  stream << "     }\n";
  stream << "     ]\n";
  stream << "}\n";

  // X Label
  stream << "Transform {\n";
  stream << "   translation " <<  .95 * len << " " << -len/40. << " " << len/ 30. << "\n";
  stream << "   children [\n";
  stream << "   Shape {\n";
  stream << "     appearance Appearance {" << std::endl;
  stream << "       material Material { " << std::endl;
  stream << "          diffuseColor 0.8 0.0 0.0 " << std::endl;
  stream << "          ambientIntensity 0.2 " << std::endl;
  stream << "          emissiveColor 0.0 0.0 0.0 " << std::endl;
  stream << "          specularColor 0.0 0.0 0.0 " << std::endl;
  stream << "          shininess 0.2 " << std::endl;
  stream << "          transparency 0.0 " << std::endl;
  stream << "       } " << std::endl;
  stream << "     } " << std::endl;
  stream << "     geometry Text {\n";
  stream << "       string \"X\"";
  stream << "       fontStyle FontStyle {\n";
  stream << "         size " <<  len /15. << "\n";
  stream << "           }\n";
  stream << "        }\n";
  stream << "       }\n";
  stream << "     ]\n";
  stream << "    }\n";

  // Y Arrow
  stream << "Transform {\n";
  stream << "   rotation 0 0 1 0\n";
  stream << "   translation 0 " <<  len << " 0\n";
  stream << "   children [\n";
  stream << "   Shape {\n";
  stream << "     appearance Appearance {" << std::endl;
  stream << "       material Material { " << std::endl;
  stream << "          diffuseColor 0.0 0.8 0.0 " << std::endl;
  stream << "          ambientIntensity 0.2 " << std::endl;
  stream << "          emissiveColor 0.0 0.0 0.0 " << std::endl;
  stream << "          specularColor 0.0 0.0 0.0 " << std::endl;
  stream << "          shininess 0.2 " << std::endl;
  stream << "          transparency 0.0 " << std::endl;
  stream << "       } " << std::endl;
  stream << "     } " << std::endl;
  stream << "     geometry Cone {\n";
  stream << "        bottomRadius " << len / 100. << "\n";
  stream << "        height " << len /10. << "\n";
  stream << "        }\n";
  stream << "     }\n";
  stream << "     ]\n";
  stream << "}\n";

  // Y Label
  stream << "Transform {\n";
  stream << "   translation " << -len/40. << " " <<  .95 * len << " " << len/ 30. << "\n";
  stream << "   children [\n";
  stream << "   Shape {\n";
  stream << "     appearance Appearance {" << std::endl;
  stream << "       material Material { " << std::endl;
  stream << "          diffuseColor 0.0 0.8 0.0 " << std::endl;
  stream << "          ambientIntensity 0.2 " << std::endl;
  stream << "          emissiveColor 0.0 0.0 0.0 " << std::endl;
  stream << "          specularColor 0.0 0.0 0.0 " << std::endl;
  stream << "          shininess 0.2 " << std::endl;
  stream << "          transparency 0.0 " << std::endl;
  stream << "       } " << std::endl;
  stream << "     } " << std::endl;
  stream << "     geometry Text {\n";
  stream << "       string \"Y\"";
  stream << "       fontStyle FontStyle {\n";
  stream << "         size " <<  len /15. << "\n";
  stream << "           }\n";
  stream << "        }\n";
  stream << "       }\n";
  stream << "     ]\n";
  stream << "    }\n";

  // Z arrow
  stream << "Transform {\n";
  stream << "   rotation 1 0 0 1.5707963 \n";
  stream << "   translation 0 0 " <<  .96 * len << " \n";
  stream << "   children [\n";
  stream << "   Shape {\n";
  stream << "     appearance Appearance {" << std::endl;
  stream << "       material Material { " << std::endl;
  stream << "          diffuseColor 0.0 0.0 0.8 " << std::endl;
  stream << "          ambientIntensity 0.2 " << std::endl;
  stream << "          emissiveColor 0.0 0.0 0.0 " << std::endl;
  stream << "          specularColor 0.0 0.0 0.0 " << std::endl;
  stream << "          shininess 0.2 " << std::endl;
  stream << "          transparency 0.0 " << std::endl;
  stream << "       } " << std::endl;
  stream << "     } " << std::endl;
  stream << "     geometry Cone {\n";
  stream << "        bottomRadius " << len / 100. << "\n";
  stream << "        height " << len /10. << "\n";
  stream << "        }\n";
  stream << "     }\n";
  stream << "     ]\n";
  stream << "}\n";

  // Z label
  stream << "Transform {\n";
  stream << "   rotation 0 1 0 -1.5707963 \n";
  stream << "   translation "  << -len/30. << " " << -len/ 40. <<  " " << .95 * len << "\n";
  stream << "   children [\n";
  stream << "   Shape {\n";
  stream << "     appearance Appearance {" << std::endl;
  stream << "       material Material { " << std::endl;
  stream << "          diffuseColor 0.0 0.0 0.8 " << std::endl;
  stream << "          ambientIntensity 0.2 " << std::endl;
  stream << "          emissiveColor 0.0 0.0 0.0 " << std::endl;
  stream << "          specularColor 0.0 0.0 0.0 " << std::endl;
  stream << "          shininess 0.2 " << std::endl;
  stream << "          transparency 0.0 " << std::endl;
  stream << "       } " << std::endl;
  stream << "     } " << std::endl;
  stream << "     geometry Text {\n";
  stream << "       string \"Z\"";
  stream << "       fontStyle FontStyle {\n";
  stream << "         size " <<  len /15. << "\n";
  stream << "           }\n";
  stream << "        }\n";
  stream << "       }\n";
  stream << "     ]\n";
  stream << "    }\n";

  // close top-level transform
  stream << "  ]\n";
  stream << "}\n";
  
  return stream;
}

#endif
