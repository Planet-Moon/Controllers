#include "JsonInterface.h"
#include <fstream>

void writeToFile(const std::string& filename, const Json::Value& obj){
    std::ofstream file_id;
    file_id.open(filename);

    Json::StyledWriter styledWriter;
    file_id << styledWriter.write(obj);

    file_id.close();
}
