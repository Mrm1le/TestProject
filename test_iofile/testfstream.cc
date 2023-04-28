#include <iostream>
#include <fstream>

int main() {
  // Open a file for writing
  std::fstream file("curve.txt", std::ios::out);
  
  if (!file.is_open()) {
    std::cout << "Failed to open the file!" << std::endl;
    return 1;
  }
  
  // Write specific information to the file
  file << "Curve Information:" << std::endl;
  file << "Curve Type: Circle" << std::endl;
  file << "Center: (0, 0)" << std::endl;
  file << "Radius: 5.0" << std::endl;
  
  // Close the file
  file.close();
  
  std::cout << "Information written to the file." << std::endl;
  
  return 0;
}
