#include <iostream>
#include <vector>
#include <bits/stdc++.h>

void Tiny2Great(std::vector<double>& vec){
    double temp_max = 0.0;
    for(auto c : vec){
        temp_max = std::max(temp_max, c);
    }
    for(int i = 0; i < vec.size(); ++i){
        vec[i] = temp_max - vec[i];
    }
}

void Standard(std::vector<std::vector<double>>& matrix){
    for(int i = 0; i < matrix.size(); ++i){
        double temp_sum = 0.0;
        for(int j = 0; j < matrix[i].size(); ++j){
            temp_sum += matrix[i][j] * matrix[i][j];
        }
        temp_sum = std::sqrt(temp_sum);
        for(int j = 0; j < matrix[i].size(); ++j){
            matrix[i][j] = matrix[i][j] / temp_sum;
        }
    }
}

double Normalized(std::vector<std::vector<double>>& matrix){
    std::vector<double> vec_max;
    std::vector<double> vec_min;
    for(int i = 0; i < matrix.size(); ++i){
        double temp_max = DBL_MIN;
        double temp_min = DBL_MAX;
        for(int j = 0; j < matrix[i].size(); ++j){
            temp_max = std::max(temp_max, matrix[i][j]);
            temp_min = std::min(temp_min, matrix[i][j]);
        }
        vec_max.emplace_back(temp_max);
        vec_min.emplace_back(temp_min);
    }
    std::vector<double> all_score;
    double sum = 0.0;
    for(int j = 0; j < matrix[0].size(); ++j){
        double D_positive = 0.0;
        double D_negative = 0.0;
        for(int i = 0; i < matrix.size(); ++i){
            double temp_positive = vec_max[i] - matrix[i][j];
            double temp_negative = vec_min[i] - matrix[i][j];
            D_positive += temp_positive * temp_positive;
            D_negative += temp_negative * temp_negative;
        }
        double temp_score = D_negative / (D_negative + D_positive);
        sum += temp_score;
        all_score.emplace_back(temp_score);
    }

    int count = 0;
    double record = all_score[all_score.size()-1];
    for(auto c : all_score){
        if(c >= record) count++;
    }
    std::cout << "This trajectory's rank is " << count <<std::endl;
    return all_score[all_score.size()-1] / sum * 880; 
}