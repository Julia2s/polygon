#include <iostream>
#include <vector>

void sieveOfEratosthenes(int maxNum, std::vector<bool>& isPrime) {
    isPrime[0] = isPrime[1] = false; 
    for (int i = 2; i * i <= maxNum; ++i) {
        if (isPrime[i]) {
            for (int j = i * i; j <= maxNum; j += i) {
                isPrime[j] = false;
            }
        }
    }
}

int main() {
    int n;
    std::cin >> n;
    std::vector<int> numbers(n);
    int maxNum = 0;

    for (int i = 0; i < n; ++i) {
        std::cin >> numbers[i];
        if (numbers[i] > maxNum) {
            maxNum = numbers[i]; 
        }
    }

    std::vector<bool> isPrime(maxNum + 1, true);
    sieveOfEratosthenes(maxNum, isPrime);

    for (int i = 0; i < n; ++i) {
        if (isPrime[numbers[i]]) {
            std::cout << numbers[i] << " ";
        }
    }
    std::cout << std::endl;

    return 0;
}