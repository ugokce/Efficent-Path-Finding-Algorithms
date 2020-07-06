#include <stdio.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <thread>
#include <mutex>

std::mutex nodeVisitMutex;
std::mutex endSearchMutex;

//A template struct which I implemented for use of position storage and calculations of GridNodes
template <typename T>
struct Vector2
{
	Vector2() = default;

	Vector2(T xValue, T yValue)
	{
		x = xValue;
		y = yValue;
	}

	inline bool operator==(Vector2<T> otherVector) 
	{
		if (otherVector.x == x && otherVector.y == y)
			return true;
		else
			return false;
	}

	inline void operator=(Vector2<T> otherVector)
	{
		x = otherVector.x;
		y = otherVector.y;
	}

	static T Distance(Vector2<T> firstPosition, Vector2<T> secondPosition)
	{
		return sqrt((firstPosition.x - secondPosition.x) * (firstPosition.x - secondPosition.x) +
		            (firstPosition.y - secondPosition.y) * (firstPosition.y - secondPosition.y));
	}

	static T ManhattanDistance(Vector2<T> firstPosition, Vector2<T> secondPosition)
	{
		return abs(firstPosition.x - secondPosition.x) + abs(firstPosition.y - secondPosition.y);
	}

	T x;
	T y;
};

//Base structure of the grid system
class GridNode
{

public:
	GridNode()
	{
		isTraversable = false;
		isVisited = false;
		distanceToTarget = std::numeric_limits<int>::max();
		localDistance = std::numeric_limits<int>::max();
		position = Vector2<float>(0, 0);
		parentNode = nullptr;
	}

	bool isTraversable = false;
	bool isVisited = false;
	int visitidThreadId = -1;

	float  distanceToTarget = std::numeric_limits<int>::max();
	float  localDistance = std::numeric_limits<int>::max();

	void SetPosition(const Vector2<float> newPosition)
	{
		position = newPosition;
	}

	Vector2<float> GetPosition() const
	{
		return position;
	}

	float GetPositionX() const
	{
		return position.x;
	}

	float GetPositionY() const
	{
		return position.y;
	}

	void AddNeighbour(GridNode* neighbour)
	{
		neighbours.push_back(neighbour);
	}

	bool isNeighbour(GridNode* possibleNeighbour)
	{
		for (auto* neighbour : neighbours)
		{
			if (neighbour->GetPosition() == possibleNeighbour->GetPosition())
			{
				return true;
			}
		}

		return false;
	}

	std::vector<GridNode*> GetNeighbours()
	{
		return neighbours;
	}

	GridNode* GetParentNode() const
	{
		return parentNode;
	}

	void SetParentNode(GridNode* newParent)
	{
		parentNode = newParent;
	}

	bool operator()(float targetDistanceA, float targetDistanceB) const
	{
		return targetDistanceA < targetDistanceB;
	}
	
private:
	std::vector<GridNode*> neighbours;
	GridNode* parentNode;
	Vector2<float> position;

};

GridNode* thread1LastNode = nullptr;
GridNode* thread2LastNode = nullptr;
bool isFinished = false;
bool isThread1Found = false;

//
bool IsObstacle(char input)
{
	return input == '\0' ? true : false;
}

//With this method, i am assuring that the next neighbour node isnt out of bounds and not and obstacle
bool IsValidGrid(const int index, const int mapSize, const unsigned char* pMap)
{
	return index >= 0 && index < mapSize && !IsObstacle(pMap[index]);
}

//Explore and initialize possible neighbours for a specific GridNode
void AssignNeighbourNodes(GridNode* gridArray, GridNode* currentNode, const int nMapWidth, const int nMapHeight, const unsigned char* pMap)
{
	const int index = currentNode->GetPositionY() * nMapWidth + currentNode->GetPositionX();

	const int rightNeighbourIndex = index + 1;
	const int leftNeighbourIndex = index - 1;
	const int upNeighbourIndex = index - nMapWidth;
	const int downNeighbourIndex = index + nMapWidth;

	const int gridSize = nMapWidth * nMapHeight;

	if (IsValidGrid(rightNeighbourIndex, gridSize, pMap) && currentNode->GetPositionX() < nMapWidth - 1)
	{
		gridArray[rightNeighbourIndex].SetPosition(Vector2<float>(currentNode->GetPositionX() + 1, currentNode->GetPositionY()));
		gridArray[rightNeighbourIndex].isTraversable = true;
		gridArray[index].AddNeighbour(&gridArray[rightNeighbourIndex]);
	}

	if (IsValidGrid(leftNeighbourIndex, gridSize, pMap) && currentNode->GetPositionX() > 0)
	{
		gridArray[leftNeighbourIndex].SetPosition(Vector2<float>(currentNode->GetPositionX() - 1, currentNode->GetPositionY()));
		gridArray[leftNeighbourIndex].isTraversable = true;
		gridArray[index].AddNeighbour(&gridArray[leftNeighbourIndex]);
	}

	if (IsValidGrid(upNeighbourIndex, gridSize, pMap) && currentNode->GetPositionY() > 0)
	{
		gridArray[upNeighbourIndex].SetPosition(Vector2<float>(currentNode->GetPositionX(), currentNode->GetPositionY() - 1));
		gridArray[upNeighbourIndex].isTraversable = true;
		gridArray[index].AddNeighbour(&gridArray[upNeighbourIndex]);
	}

	if (IsValidGrid(downNeighbourIndex, gridSize, pMap) && currentNode->GetPositionY() < nMapHeight - 1)
	{
		gridArray[downNeighbourIndex].SetPosition(Vector2<float>(currentNode->GetPositionX(), currentNode->GetPositionY() + 1));
		gridArray[downNeighbourIndex].isTraversable = true;
		gridArray[index].AddNeighbour(&gridArray[downNeighbourIndex]);
	}
}

int getPath(GridNode* startNode, GridNode* targetNode, int* pOutBuffer, const int nMapWidth)
{
	int gridStepCount = 0;
	auto* parentNode = targetNode;
	GridNode* targetLastNode = parentNode;

	while (true)
	{
		targetLastNode = targetLastNode->GetParentNode();

		if (!targetLastNode || targetLastNode->GetParentNode() == nullptr)
		{
			break;
		}
	}

	const int nodeArrayIndex = targetLastNode->GetPositionY() * nMapWidth + targetLastNode->GetPositionX();
	printf("targetLastNode = %d \n", nodeArrayIndex);

	GridNode* startParentNode;

	if (isThread1Found)
	{
		startParentNode = thread1LastNode;
	}
	else
	{
		startParentNode = thread2LastNode;
	}

	const int nodeArrayIndex2 = startParentNode->GetPositionY() * nMapWidth + startParentNode->GetPositionX();
	printf("startParentNode = %d \n", nodeArrayIndex2);

	while (startParentNode != nullptr)
	{
		bool isNeighbour = targetLastNode->isNeighbour(startParentNode);

		if (isNeighbour || startParentNode == nullptr)
		{
			break;
		}

		startParentNode = startParentNode->GetParentNode();
	}

	const int nodeArrayIndex3 = startParentNode->GetPositionY() * nMapWidth + startParentNode->GetPositionX();
	printf("startParentNode = %d \n", nodeArrayIndex3);

	targetLastNode->SetParentNode(startParentNode);


	while (parentNode != nullptr)
	{
		if (parentNode == startNode || parentNode == parentNode->GetParentNode())
		{
			break;
		}

		const int nodeArrayIndex = parentNode->GetPositionY() * nMapWidth + parentNode->GetPositionX();

		pOutBuffer[gridStepCount] = nodeArrayIndex;

		gridStepCount++;
		parentNode = parentNode->GetParentNode();

		//if (parentNode == nullptr)
		//{
			//pOutBuffer[gridStepCount] = 0;
			//gridStepCount = 0;
			//return -1;
		//}
	}

	std::reverse(pOutBuffer, pOutBuffer + gridStepCount);

	return gridStepCount;
}

void UpdateNeighbours(GridNode* currentNode, GridNode* targetNode, std::vector<GridNode*>& unTestedNodes, const int threadId)
{
	for (auto* neighbourNode : currentNode->GetNeighbours())
	{
		if (neighbourNode->visitidThreadId == -1)
		{
			if (!neighbourNode->isVisited && neighbourNode->isTraversable)
				unTestedNodes.push_back(neighbourNode);

			float calculatedLocalDistance = currentNode->localDistance + Vector2<float>::ManhattanDistance(currentNode->GetPosition(),
				neighbourNode->GetPosition());

			if (calculatedLocalDistance < neighbourNode->localDistance)
			{
				nodeVisitMutex.lock();
				
				if(threadId == 2)
				{
					currentNode->SetParentNode(neighbourNode);
				}
				else
				{
					neighbourNode->SetParentNode(currentNode);
				}
	
				neighbourNode->localDistance = calculatedLocalDistance;

				const float heuristic = Vector2<float>::ManhattanDistance(neighbourNode->GetPosition(),
					targetNode->GetPosition());

				neighbourNode->distanceToTarget = neighbourNode->localDistance + heuristic;

				nodeVisitMutex.unlock();
			}
		}
	}
}



void RunAStar(GridNode* gridArray, GridNode* startNode, GridNode* targetNode, Vector2<int> mapSize,
	const unsigned char* pMap, const int threadId)
{
	GridNode* currentNode = startNode;
	startNode->localDistance = 0.0f;
	startNode->distanceToTarget = Vector2<float>::ManhattanDistance(startNode->GetPosition(), targetNode->GetPosition());

	std::vector<GridNode*> unTestedNodes;

	unTestedNodes.push_back(startNode);

	while (!unTestedNodes.empty() && currentNode != targetNode && !isFinished)
	{
		std::sort(unTestedNodes.begin(), unTestedNodes.end());

		while (!unTestedNodes.empty() && unTestedNodes.front()->isVisited)
		{
			unTestedNodes.erase(unTestedNodes.begin());
		}

		endSearchMutex.lock();
		if ((unTestedNodes.empty() && !isFinished) || (currentNode->visitidThreadId != -1 && currentNode->visitidThreadId != threadId && !isFinished))
		{
			isFinished = true;

			if (threadId == 1)
			{
				thread1LastNode = currentNode;
				isThread1Found = true;

				printf("ilk thread 1 bitti \n");
			}
			else
			{
				thread2LastNode = currentNode;
				isThread1Found = false;

				printf("ilk thread 2 bitti \n");
			}

		
			endSearchMutex.unlock();
			break;
		}
		endSearchMutex.unlock();

		printf("thread % d , current: (%f , %f) \n", threadId, currentNode->GetPositionX(), currentNode->GetPositionY());

		currentNode = unTestedNodes.front();
		currentNode->isVisited = true;

		if (isFinished)
		{
			break;
		}

		if (currentNode->visitidThreadId == -1)
		{
			currentNode->visitidThreadId = threadId;
			AssignNeighbourNodes(gridArray, currentNode, mapSize.x, mapSize.y, pMap);
			UpdateNeighbours(currentNode, targetNode, unTestedNodes, threadId);
		}
	}
}

std::vector<GridNode*> deneme;

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap,
	const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize)
{
	GridNode* gridArray = new GridNode[nMapWidth * nMapHeight];
	const Vector2<int> mapSize(nMapWidth, nMapHeight);

	for (int i = 0; i < nMapWidth * nMapHeight; i++)
	{
		deneme.push_back(&gridArray[i]);
	}

	GridNode* startNode = &gridArray[nStartY * nMapWidth + nStartX];
	startNode->SetPosition(Vector2<float>(nStartX, nStartY));

	GridNode* targetNode = &gridArray[nTargetY * nMapWidth + nTargetX];
	targetNode->SetPosition(Vector2<float>(nTargetX, nTargetY));

	if (startNode->GetPosition() == targetNode->GetPosition())
	{
		return 0;
	}

	std::thread firstThread(RunAStar, gridArray, startNode, targetNode, mapSize, pMap, 1);
	std::thread	secondThread(RunAStar, gridArray, targetNode, startNode, mapSize, pMap, 2);

	firstThread.join();
	secondThread.join();

	return getPath(startNode, targetNode, pOutBuffer, nMapWidth);
}

int main()
{
	unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
	int pOutBuffer[12];

	const int counter = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);

	for (int i = 0; i < counter; i++)
	{
		printf("%d, ", pOutBuffer[i]);
	}


	return 0;
}

