#ifndef KMALGORITHM_H
#define KMALGORITHM_H

#include <iostream>
#include <fstream>
#include <third_party/Hungarian.hpp>
#include <algorithm>
#include "ellipse.hpp"

class KMAlgorithm
{
public:
	KMAlgorithm()
	{
		unused_clear_time = 5;
		unused_label.reserve(100);
		for (int i = 100; i > 0; i--)
			unused_label.push_back(i);
	}
	void tracking(std::vector<Ellipse> &input_vector);

private:
	void check_in_his_list(Ellipse &input, std::unordered_map<int, Ellipse>& used_label);
	void update_his_list(std::unordered_map<int, Ellipse>& used_label);

	std::vector<Ellipse> last_label_list; 			// 上一次tracking时的输入 input_vector
	std::vector<pair<Ellipse, int>> old_label_list; // second用来表示对应的Ellipse未使用次数
	std::vector<int> unused_label;					// 表示未使用的label

	int unused_clear_time;
};

void KMAlgorithm::tracking(std::vector<Ellipse> &input_vector)
{
	int new_size = input_vector.size();
	int last_size = last_label_list.size();
	int his_size = old_label_list.size();

	std::unordered_map<int, Ellipse> used_label;
	std::vector<int> unassigned_input_idx;

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
	}
	else if (last_size == 0) 
	{
		std::cout << "last obs empty" << std::endl;
		for (auto &input : input_vector) // 在历史obs中寻找与当前obs对应的
			check_in_his_list(input, used_label);
	}
	else if (new_size > 0)
	{
		vector<vector<double>> dis(new_size, vector<double>(last_size)); // 计算本次检测到的obs与上次检测到的obs的距离

		for (int i = 0; i < new_size; i++)
		{
			for (int j = 0; j < last_size; j++)
			{
				dis[i][j] = calculate_dis(input_vector[i], last_label_list[j]);
			}
		}

		HungarianAlgorithm hun_alg;
		vector<int> assignment;
		double cost = hun_alg.Solve(dis, assignment);

		for (size_t i = 0; i < new_size; i++)	// 根据最优匹配的结果给本次检测到的obs添加label
		{
			if (assignment[i] != -1 && calculate_dis(input_vector[i], last_label_list[assignment[i]]) < 1)
			{
				input_vector[i].label = last_label_list[assignment[i]].label; 		// 记录已经使用的label，避免重复分配
				used_label[input_vector[i].label] = input_vector[i];		
			}
			else
			{
				unassigned_input_idx.push_back(i);									// 记录未分配的cylinder的idx
			}
		}

		for	(size_t i = 0; i < unassigned_input_idx.size(); i++)
		{
			check_in_his_list(input_vector[unassigned_input_idx[i]], used_label);
		}
	}
	// update last_list
	last_label_list = input_vector;

	update_his_list(used_label);
}

void KMAlgorithm::update_his_list(std::unordered_map<int, Ellipse>& used_label)
{
	int his_size = old_label_list.size();
	vector<int> remove_list;

	for(int j = 0; j < his_size; j++)
	{
		if(used_label.find(old_label_list[j].first.label) != used_label.end())
		{
			old_label_list[j].second = 0;
			old_label_list[j].first = used_label[old_label_list[j].first.label];
		}
	}

	for (int j = 0; j < his_size; j++)
	{
		old_label_list[j].second++;
		if (old_label_list[j].second > unused_clear_time)
		{
			remove_list.push_back(old_label_list[j].first.label);
			continue;
		}
	}

	old_label_list.erase(std::remove_if(old_label_list.begin(), old_label_list.end(),
										[&](const pair<Ellipse, int> &p)
										{ return p.second > unused_clear_time; }),
						 old_label_list.end());

	for (int i : remove_list)
		unused_label.push_back(i);
}

void KMAlgorithm::check_in_his_list(Ellipse &input, std::unordered_map<int, Ellipse>& used_label)
{
	int his_size = old_label_list.size();
	int index_track = 0;
	float dis;
	float min_dis = 10;
	for (int j = 0; j < his_size; j++)
	{
		if(used_label.find(old_label_list[j].first.label) != used_label.end())	// 已经使用过这个label
			continue;

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
	}
	else // 历史标签中没有匹配的，创建一个新的标签
	{
		input.label = unused_label.back();
		unused_label.pop_back();
		pair<Ellipse, int> cur_Ellipse = pair<Ellipse, int>(input, 0);
		old_label_list.push_back(cur_Ellipse);
	}
	used_label[input.label] = input;
}

#endif