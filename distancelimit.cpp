#include "header.h"

double distancelimit() //거리입력
{
    double return_value = 0;

    std::cout << "Input exit distance (>1 Meter) : ";
    std::cin >> return_value;

    if(return_value < 1)
    {
      std::string message = "input_error";
      error_Handle(message);
    }

    return return_value;
}
