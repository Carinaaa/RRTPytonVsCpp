#pragma once
#include <tuple>
#include <array>
#include <memory>
#include <list>

struct Node {
	std::tuple<int, int> x_y{}; // change to Point from lib opencv
	std::shared_ptr<Node> parent = std::make_shared<Node>();
};

typedef std::shared_ptr<Node> NodePtr;


class RRTGraphCpp {

public:
	NodePtr start{};
	NodePtr goal{};
	int obsdim{};
	int obsnum{};
	int map_height{};
	int map_weight{};
	bool goalFlag{ false };
	std::tuple<std::array<int, 30>, std::array<int, 30>> goalState{ {0},{0} };
	std::tuple<std::array<int, 30>, std::array<int, 30>> path{ {0},{0} };
	std::tuple<std::array<int, 30>, std::array<int, 30>> obstacles{ {0},{0} };

	RRTGraphCpp(
		NodePtr start,
		NodePtr goal,
		std::pair<int, int> MapDimensions,
		int obsdim,
		int obsnum) :
		start(start), goal(goal),
		map_height(MapDimensions.first), map_weight(MapDimensions.second),
		obsdim(obsdim), obsnum(obsnum)
	{

	}

	std::pair<int, int> makeRandomRRect() {
		return std::pair<int, int>{rand() % (map_height-obsdim), rand() % (map_weight - obsdim)};
	}

	//std::list<std::tuple<int, int>> makeObs() {
	//	std::list<std::tuple<int, int>> obstacles{};
	//	for (auto i = 0; i < obsnum; ++i) {

	//	}
	//	return obstacles;
	//}

};