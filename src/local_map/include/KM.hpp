#ifndef KMALGORITHM_H
#define KMALGORITHM_H

#include <iostream>
#include <fstream>
#include <third_party/Hungarian.hpp>
#include <algorithm>
#include "ellipse.hpp"

// 用一个map记录当前正在使用的label
// 记录每个label上一次使用的时间
// 假设上一次使用的时间超出阈值，这个label设置为可用

class KMAlgorithm
{
public:
	KMAlgorithm()
	{
		unused_label.reserve(100);
		for (int i = 1; i <= 100; ++i)
			unused_label.push_back(i);
	}
	void tracking(std::vector<Ellipse> &input_vector);

private:
	void check_in_his_list(Ellipse &obs);

	std::vector<Ellipse> last_label_list; // 上一次tracking时的输入 input_vector
	// std::vector<Ellipse> old_label_list;	//
	std::vector<pair<Ellipse, int>> old_label_list; // second用来表示对应的Ellipse未使用次数
	std::vector<int> unused_label;					// 表示未使用的label

	std::ofstream pose_file;
};

void KMAlgorithm::tracking(std::vector<Ellipse> &input_vector)
{
	int new_size = input_vector.size();
	int last_size = last_label_list.size();
	int his_size = old_label_list.size();

	if (his_size == 0) // 历史obs数量为 0
	{
		std::cout << "his obs empty" << std::endl;
		for (size_t i = 0; i < input_vector.size(); i++)
		{
			input_vector[i].label = unused_label.back(); // label从1开始
			unused_label.pop_back();
			pair<Ellipse, int> cur_Ellipse = pair<Ellipse, int>(input_vector[i], 0);
			old_label_list.push_back(cur_Ellipse);
		}
		// old_label_list = input_vector;		// input直接加入历史obs
	}
	else if (last_size == 0) // 上一次tracking时没有检测到obs，应该是处理上一次漏检的情况
	{
		std::cout << "last obs empty" << std::endl;
		for (auto &input : input_vector) // 在历史obs中寻找与当前obs对应的
			check_in_his_list(input);
	}
	else if (new_size > 0)
	{
		std::cout << "cal cur pre obs dis" << std::endl;
		vector<vector<double>> dis(new_size, vector<double>(last_size)); // 计算本次检测到的obs与上次检测到的obs的距离
		pose_file.open("/home/snuc/CZM/problem.txt", std::ios::out | std::ios::app);
		for (int i = 0; i < new_size; i++)
		{
			for (int j = 0; j < last_size; j++)
			{
				dis[i][j] = calculate_dis(input_vector[i], last_label_list[j]);
				pose_file << dis[i][j] << " "
			}
			pose_file << std::endl;
		}
		pose_file << "solved" << std::endl;
		pose_file.close();

		HungarianAlgorithm hun_alg;
		vector<int> assignment;
		std::cout << "start solve hun_alg" << std::endl;
		double cost = hun_alg.Solve(dis, assignment);
		std::cout << "hun_alg solved" << std::endl;
		for (size_t i = 0; i < new_size; i++)
		{
			if (assignment[i] != -1 && calculate_dis(input_vector[i], last_label_list[assignment[i]]) < 1)
				input_vector[i].label = last_label_list[assignment[i]].label; // 根据最优匹配的结果给本次检测到的obs添加label
			else
				check_in_his_list(input_vector[i]);
		}
	}
	// update last_list
	last_label_list = input_vector;
}

void KMAlgorithm::check_in_his_list(Ellipse &input)
{
	vector<int> remove_list;
	int his_size = old_label_list.size();
	int index_track = 0;
	float dis;
	float min_dis = 10;
	for (int j = 0; j < his_size; j++)
	{
		old_label_list[j].second++;
		if (old_label_list[j].second > 10)
		{
			remove_list.push_back(old_label_list[j].first.label);
			continue;
		}

		dis = calculate_dis(input, old_label_list[j].first);
		if (dis < min_dis)
		{
			min_dis = dis;
			index_track = j;
		}
	}

	if (min_dis < 0.2)
	{
		input.label = old_label_list[index_track].first.label;
		old_label_list[index_track].first = input; // 更新Ellipse的属性
		old_label_list[index_track].second = 0;
	}
	else // 历史标签中没有匹配的，创建一个新的标签
	{
		input.label = unused_label.back();
		unused_label.pop_back();
		pair<Ellipse, int> cur_Ellipse = pair<Ellipse, int>(input, 0);
		old_label_list.push_back(cur_Ellipse);
	}

	old_label_list.erase(std::remove_if(old_label_list.begin(), old_label_list.end(),
										[](const pair<Ellipse, int> &p)
										{ return p.second > 10; }),
						 old_label_list.end());

	for (int i : remove_list)
		unused_label.push_back(i);
}

#endif