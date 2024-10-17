#ifndef KMALGORITHM_H
#define KMALGORITHM_H

#include <third_party/Hungarian.hpp>

#include "ellipse.hpp"

class KMAlgorithm
{
public:
	void tracking(std::vector<Ellipse> &input_vector);

private:
	void check_in_his_list(Ellipse &obs);

	std::vector<Ellipse> last_label_list;	// 上一次tracking时的输入 input_vector
	std::vector<Ellipse> old_label_list;	// 
};

void KMAlgorithm::tracking(std::vector<Ellipse> &input_vector)
{
	int new_size = input_vector.size();
	int last_size = last_label_list.size();
	int his_size = old_label_list.size();

	if (his_size == 0)	// 历史obs数量为 0
	{
		for (size_t i = 0; i < input_vector.size(); i++)
			input_vector[i].label = i + 1;	// label从1开始
		old_label_list = input_vector;		// input直接加入历史obs
	}
	else if (last_size == 0)				// 上一次tracking时没有检测到obs，应该是处理上一次漏检的情况
	{
		for (auto &input : input_vector)	// 在历史obs中寻找与当前obs对应的
			check_in_his_list(input);
	}
	else if (new_size > 0)
	{
		vector<vector<double>> dis(new_size, vector<double>(last_size));	// 计算本次检测到的obs与上次检测到的obs的距离
		for (int i = 0; i < new_size; i++)
			for (int j = 0; j < last_size; j++)
				dis[i][j] = calculate_dis(input_vector[i], last_label_list[j]);

		HungarianAlgorithm hun_alg;
		vector<int> assignment;
		double cost = hun_alg.Solve(dis, assignment);
		for (size_t i = 0; i < new_size; i++)
		{
			if (assignment[i] != -1 && calculate_dis(input_vector[i], last_label_list[assignment[i]]) < 1)
				input_vector[i].label = last_label_list[assignment[i]].label;	// 根据最优匹配的结果给本次检测到的obs添加label
			else
				check_in_his_list(input_vector[i]);
		}
	}
	// update last_list
	last_label_list = input_vector;
}

void KMAlgorithm::check_in_his_list(Ellipse &input)
{
	int his_size = old_label_list.size();
	int index_track;
	float dis;
	float min_dis = 10;
	for (int j = 0; j < his_size; j++)
	{
		dis = calculate_dis(input, old_label_list[j]);
		if (dis < min_dis)
		{
			min_dis = dis;
			index_track = j;
		}
	}

	if (min_dis < 0.2)
	{
		input.label = old_label_list[index_track].label;
		old_label_list[index_track] = input;
	}
	else
	{
		input.label = old_label_list.size() + 1;
		old_label_list.emplace_back(input);
	}
}

#endif