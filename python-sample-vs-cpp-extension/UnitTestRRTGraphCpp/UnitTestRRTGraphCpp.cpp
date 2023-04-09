#include "pch.h"
#include "CppUnitTest.h"
#include "module.cpp"
#include "RRTGraphCpp.h"
#include <Python.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace ModuleCpp
{
	TEST_CLASS(ModuleCpp)
	{
	public:

		TEST_METHOD(TestMethod_Sin)
		{
			Assert::AreEqual(sinh_impl(-30), -sinh_impl(30));
		}

		TEST_METHOD(TestMethod_Cos)
		{
			Assert::AreEqual(cosh_impl(-30), cosh_impl(30));

		}

		TEST_METHOD(TestMethod_Tan)
		{
		//	double reasult = PyFloat_AsDouble(tanh_impl({}, PyFloat_FromDouble(double{ 45 })));
		//	Assert::AreEqual(reasult, double{ 1 });
			
		}
	};
}

namespace UnitTestRRTGraphCpp
{
	TEST_CLASS(UnitTestRRTGraphCpp)
	{
	public:
		TEST_METHOD(TestGraph_makeRandomRRect) {
		/*	std::tuple<int, int> start_coordinates{ 50, 50 };
			std::tuple<int, int> goal_coordinates{ 510, 510 };

			NodePtr start = std::make_shared<Node>();
			start->x_y = start_coordinates;
			NodePtr goal = std::make_shared<Node>();
			goal->x_y = goal_coordinates;

			RRTGraphCpp graph{ start, goal, 
				std::pair<int, int>{30,50}, 600, 100 };

			auto iteration{ 2 };

			while (iteration) {
				std::pair<int, int> result = graph.makeRandomRRect();

				if ((result.first >= 0) && (result.first <= (graph.map_height - graph.obsdim)))
				{
					Assert::IsTrue(true);
				}
				else {
					Assert::IsTrue(false);
				}

				if ((result.second >= 0) && (result.second <= (graph.map_weight - graph.obsdim)))
				{
					Assert::IsTrue(true);
				}
				else {
					Assert::IsTrue(false);
				}
				--iteration;
			}*/
		}
		
	};
}
