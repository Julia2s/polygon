#include <iostream>
#include <vector>
#include <cassert>

using namespace std;

void heapify(int arr[], int n, int i) {
    int largest = i;
    int left = 2 * i + 1;
    int right = 2 * i + 2;

    if (left < n && arr[left] > arr[largest])
        largest = left;

    if (right < n && arr[right] > arr[largest])
        largest = right;

    if (largest != i) {
        swap(arr[i], arr[largest]);
        heapify(arr, n, largest);
    }
}

void heapSort(int arr[], int n) {
    for (int i = n / 2 - 1; i >= 0; i--) {
        heapify(arr, n, i);
    }

    for (int i = n - 1; i > 0; i--) {
        swap(arr[0], arr[i]);
        heapify(arr, i, 0);
    }
}

void printArray(int arr[], int size) {
    for (int i = 0; i < size; ++i)
        cout << arr[i] << " ";
    cout << endl;
}

void testHeapSort() {
    // Тест 1: Пустой массив
    int arr1[] = {};
    int n1 = sizeof(arr1) / sizeof(arr1[0]);
    heapSort(arr1, n1);
    assert(n1 == 0); // Пустой массив

    // Тест 2: Массив из одного элемента
    int arr2[] = {1};
    int n2 = sizeof(arr2) / sizeof(arr2[0]);
    heapSort(arr2, n2);
    assert(arr2[0] == 1); // Ожидаем: 1

    // Тест 3: Массив, уже отсортированный
    int arr3[] = {1, 2, 3, 4, 5};
    int n3 = sizeof(arr3) / sizeof(arr3[0]);
    heapSort(arr3, n3);
    assert(arr3[0] == 1 && arr3[1] == 2 && arr3[2] == 3 && arr3[3] == 4 && arr3[4] == 5); // Ожидаем: 1 2 3 4 5

    // Тест 4: Массив, отсортированный в обратном порядке
    int arr4[] = {5, 4, 3, 2, 1};
    int n4 = sizeof(arr4) / sizeof(arr4[0]);
    heapSort(arr4, n4);
    assert(arr4[0] == 1 && arr4[1] == 2 && arr4[2] == 3 && arr4[3] == 4 && arr4[4] == 5); // Ожидаем: 1 2 3 4 5

    // Тест 5: Случайный массив
    int arr5[] = {3, 6, 1, 8, 4, 5};
    int n5 = sizeof(arr5) / sizeof(arr5[0]);
    heapSort(arr5, n5);
    assert(arr5[0] == 1 && arr5[1] == 3 && arr5[2] == 4 && arr5[3] == 5 && arr5[4] == 6 && arr5[5] == 8); 

    // Тест 6: Массив с повторяющимися элементами
    int arr6[] = {3, 3, 2, 1, 2, 1, 3};
    int n6 = sizeof(arr6) / sizeof(arr6[0]);
    heapSort(arr6, n6);
    assert(arr6[0] == 1 && arr6[1] == 1 && arr6[2] == 2 && arr6[3] == 2 && arr6[4] == 3 && arr6[5] == 3 && arr6[6] == 3); 

    cout << "Все тесты пройдены успешно!" << endl;
}

int main() {
    testHeapSort();
    return 0;
}


