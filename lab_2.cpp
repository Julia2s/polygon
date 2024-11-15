#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>

int main() {
    int size; 
    std::cout << "size = ";
    std::cin >> size;

    std::vector<std::string> arr(size); 
    std::cout << "arr = ";
    for (int i = 0; i < size; ++i) {
        std::cin >> arr[i];
    }

    // Хранение кол-ва уникальных значений
    std::unordered_map<std::string, int> countMap;

    // Подсчет кол-ва уникальных значений
    for (const std::string& value : arr) {
        countMap[value]++;
    }

    // Замена значений в массиве на количество их повторений
    std::vector<int> res(size);
    for (int i = 0; i < size; ++i) {
        res[i] = countMap[arr[i]];
    }

    for (int i = 0; i < res.size(); ++i) {
        std::cout << res[i];
        if (i < res.size() - 1) {
            std::cout << ", ";
        }
    }

    return 0;
}

