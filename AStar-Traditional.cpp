#include <stdio.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>

#define Weight 10;

template <typename T>
struct Vector2
{
    Vector2() = default;

    Vector2(T xValue, T yValue)
    {
        x = xValue;
        y = yValue;
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
struct GridNode
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

    float  distanceToTarget;
    float  localDistance;

    void SetPosition(const Vector2<float> newPosition)
    {
        position = newPosition;
    }

    Vector2<float> GetPosition()
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
void AssignNeighbourNodes(GridNode* gridNodes, GridNode* currentNode, const int nMapWidth, const int nMapHeight, const unsigned char* pMap)
{
    const int index = currentNode->GetPositionY() * nMapWidth + currentNode->GetPositionX();

    const int rightNeighbourIndex = index + 1;
    const int leftNeighbourIndex = index - 1;
    const int upNeighbourIndex = index - nMapWidth;
    const int downNeighbourIndex = index + nMapWidth;

    const int gridSize = nMapWidth * nMapHeight;

    if (IsValidGrid(rightNeighbourIndex, gridSize, pMap) && currentNode->GetPositionX() < nMapWidth - 1)
    {
        gridNodes[rightNeighbourIndex].SetPosition(Vector2<float>(currentNode->GetPositionX() + 1, currentNode->GetPositionY()));
        gridNodes[rightNeighbourIndex].isTraversable = true;
        gridNodes[index].AddNeighbour(&gridNodes[rightNeighbourIndex]);
    }

    if (IsValidGrid(leftNeighbourIndex, gridSize, pMap) && currentNode->GetPositionX() > 0)
    {
        gridNodes[leftNeighbourIndex].SetPosition(Vector2<float>(currentNode->GetPositionX() - 1, currentNode->GetPositionY()));
        gridNodes[leftNeighbourIndex].isTraversable = true;
        gridNodes[index].AddNeighbour(&gridNodes[leftNeighbourIndex]);
    }

    if (IsValidGrid(upNeighbourIndex, gridSize, pMap) && currentNode->GetPositionY() > 0)
    {
        gridNodes[upNeighbourIndex].SetPosition(Vector2<float>(currentNode->GetPositionX(), currentNode->GetPositionY() - 1));
        gridNodes[upNeighbourIndex].isTraversable = true;
        gridNodes[index].AddNeighbour(&gridNodes[upNeighbourIndex]);
    }

    if (IsValidGrid(downNeighbourIndex, gridSize, pMap) && currentNode->GetPositionY() < nMapHeight - 1)
    {
        gridNodes[downNeighbourIndex].SetPosition(Vector2<float>(currentNode->GetPositionX(), currentNode->GetPositionY() + 1));
        gridNodes[downNeighbourIndex].isTraversable = true;
        gridNodes[index].AddNeighbour(&gridNodes[downNeighbourIndex]);
    }
}

int GetPath(GridNode* startNode, GridNode* targetNode, int* pOutBuffer, const int nMapWidth)
{
    int gridStepCount = 0;
    auto* parentNode = targetNode;

    while (parentNode != nullptr)
    {
        if (parentNode == startNode)
        {
            break;
        }

        const int nodeArrayIndex = parentNode->GetPositionY() * nMapWidth + parentNode->GetPositionX();

        pOutBuffer[gridStepCount] = nodeArrayIndex;

        gridStepCount++;
        parentNode = parentNode->GetParentNode();

        if (parentNode == nullptr)
        {
            pOutBuffer[gridStepCount] = 0;
            gridStepCount = 0;
            return -1;
        }
    }

    std::reverse(pOutBuffer, pOutBuffer + gridStepCount);

    return gridStepCount;
}

void UpdateNeighbours(GridNode* currentNode, GridNode* targetNode, std::vector<GridNode*>& unTestedNodes)
{
    for (auto* neighbourNode : currentNode->GetNeighbours())
    {
        if (!neighbourNode->isVisited && neighbourNode->isTraversable)
            unTestedNodes.push_back(neighbourNode);

        float calculatedLocalDistance = currentNode->localDistance + Vector2<float>::ManhattanDistance(currentNode->GetPosition(),
            neighbourNode->GetPosition());

        if (calculatedLocalDistance < neighbourNode->localDistance)
        {
            neighbourNode->SetParentNode(currentNode);
            neighbourNode->localDistance = calculatedLocalDistance;

            const float heuristic = Vector2<float>::ManhattanDistance(neighbourNode->GetPosition(),
                targetNode->GetPosition());

            neighbourNode->distanceToTarget = neighbourNode->localDistance + heuristic * Weight;
        }
    }
}

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap,
    const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize)
{
    GridNode* gridNodes = new GridNode[nMapWidth * nMapHeight];

    GridNode* startNode = &gridNodes[nStartY * nMapWidth + nStartX];
    startNode->SetPosition(Vector2<float>(nStartX, nStartY));

    GridNode* targetNode = &gridNodes[nTargetY * nMapWidth + nTargetX];
    targetNode->SetPosition(Vector2<float>(nTargetX, nTargetY));

    if (startNode == targetNode)
    {
        return 0;
    }

    GridNode* currentNode = startNode;
    startNode->localDistance = 0.0f;
    startNode->distanceToTarget = Vector2<float>::ManhattanDistance(startNode->GetPosition(), targetNode->GetPosition());

    std::vector<GridNode*> unTestedNodes;
    unTestedNodes.push_back(startNode);

    while (!unTestedNodes.empty() && currentNode != targetNode)
    {
        std::sort(unTestedNodes.begin(), unTestedNodes.end());
        
        while (!unTestedNodes.empty() && unTestedNodes.front()->isVisited)
        {
            unTestedNodes.erase(unTestedNodes.begin());
        }
            
        if(unTestedNodes.empty())
            break;

        currentNode = unTestedNodes.front();
        currentNode->isVisited = true; 

        AssignNeighbourNodes(gridNodes, currentNode, nMapWidth, nMapHeight, pMap);
        UpdateNeighbours(currentNode, targetNode, unTestedNodes);
    }

    gridNodes = nullptr;
    unTestedNodes.clear();

    return GetPath(startNode, targetNode, pOutBuffer, nMapWidth);
}