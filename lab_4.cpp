#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>
#include <numeric>

double best_metric;
std::vector<std::vector<double>> final_clusters;
int k;  // Количество кластеров

// Функция вычисления метрики для кластера
double compute_cluster_metric(const std::vector<double>& cluster) {
    if (cluster.empty()) return 0.0;

    double mean = std::accumulate(cluster.begin(), cluster.end(), 0.0) / cluster.size();
    double metric = 0.0;

    for (double value : cluster) {
        metric += std::abs(value - mean);
    }

    return metric;
}

// Рекурсивная функция для генерации кластеров
void generate_clusters(const std::vector<double>& data, int index, 
                       std::vector<std::vector<double>>& clusters) {
    if (index == data.size()) {
        if (clusters.size() == k) {
            double current_metric = 0.0;
            for (const auto& cluster : clusters) {
                current_metric += compute_cluster_metric(cluster);
            }
            if (current_metric < best_metric) {
                best_metric = current_metric;
                final_clusters = clusters;
            }
        }
        return;
    }

    for (size_t i = 0; i < clusters.size(); ++i) {
        clusters[i].push_back(data[index]);
        generate_clusters(data, index + 1, clusters);
        clusters[i].pop_back();
    }

    if (clusters.size() < k) {
        clusters.push_back({data[index]});
        generate_clusters(data, index + 1, clusters);
        clusters.pop_back();
    }
}

// Функция для проверки результатов
void check_results(const std::vector<std::vector<double>>& actual, 
                   const std::vector<std::vector<double>>& expected, 
                   const std::string& test_name) {
    if (actual == expected) {
        std::cout << test_name << " - Passed\n";
    } else {
        std::cout << test_name << " - Failed\n";
    }
}

// Функция для запуска теста
void run_test(const std::vector<double>& data, int cluster_count, 
              const std::vector<std::vector<double>>& expected, 
              const std::string& test_name) {
    k = cluster_count;  // Используем короткую переменную для количества кластеров
    best_metric = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> clusters;

    auto start_time = std::chrono::high_resolution_clock::now();
    generate_clusters(data, 0, clusters);
    auto end_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << test_name << " - Execution time: " << duration.count() << " seconds\n";

    check_results(final_clusters, expected, test_name);
}

int main() {
    // Тест 1: Массив из 7 элементов, делим на 7 кластеров (по одному элементу в каждом)
    std::vector<double> input1 = {1.5, 3.2, -2.4, 7.1, -5.8, 4.3, 0.9};
    std::vector<std::vector<double>> expected1 = {{1.5}, {3.2}, {-2.4}, {7.1}, {-5.8}, {4.3}, {0.9}};
    run_test(input1, 7, expected1, "Test 1: 7 elements into 7 clusters");

    // Тест 2: Массив из 10 элементов, делим на 7 кластеров
    std::vector<double> input2 = {1.5, 3.2, -2.4, 7.1, -5.8, 4.3, 0.9, -1.2, 8.5, 2.0};
    std::vector<std::vector<double>> expected2 = {{1.5, 3.2}, {-2.4, 7.1}, {-5.8, 4.3}, {0.9}, {-1.2}, {8.5}, {2.0}};
    run_test(input2, 7, expected2, "Test 2: 10 elements into 7 clusters");

    // Тест 3: Массив из 15 элементов, делим на 7 кластеров
    std::vector<double> input3 = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.1, 11.2, 12.3, 13.4, 14.5, 15.6};
    std::vector<std::vector<double>> expected3 = {{1.1, 2.2}, {3.3, 4.4}, {5.5, 6.6}, {7.7, 8.8}, {9.9, 10.1}, {11.2, 12.3}, {13.4, 14.5, 15.6}};
    run_test(input3, 7, expected3, "Test 3: 15 elements into 7 clusters");

    // Тест 4: Массив из 20 элементов, делим на 7 кластеров
    std::vector<double> input4 = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.1, 11.2, 12.3, 13.4, 14.5, 15.6, 16.7, 17.8, 18.9, 19.0, 20.1};
    std::vector<std::vector<double>> expected4 = {{1.1, 2.2}, {3.3, 4.4}, {5.5, 6.6}, {7.7, 8.8}, {9.9, 10.1}, {11.2, 12.3}, {13.4, 14.5, 15.6, 16.7, 17.8}};
    run_test(input4, 7, expected4, "Test 4: 20 elements into 7 clusters");

    // Тест 5: Массив из 25 элементов, делим на 7 кластеров
    std::vector<double> input5 = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.1, 11.2, 12.3, 13.4, 14.5, 15.6, 16.7, 17.8, 18.9, 19.0, 20.1, 21.2, 22.3, 23.4, 24.5, 25.6};
    std::vector<std::vector<double>> expected5 = {{1.1, 2.2}, {3.3, 4.4}, {5.5, 6.6}, {7.7, 8.8}, {9.9, 10.1}, {11.2, 12.3}, {13.4, 14.5, 15.6, 16.7, 17.8, 18.9, 19.0}};
    run_test(input5, 7, expected5, "Test 5: 25 elements into 7 clusters");

    return 0;
}

