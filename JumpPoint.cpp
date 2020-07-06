#include <stdio.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <list>
#include <map>

bool isForced = false;

struct Node
{
public:
    void setPositon(const int xPosition, const int yPosition)
    {
        x = xPosition;
        y = yPosition;
    }

    Node()
    {
        x = 0;
        y = 0;

        heuristicCost = 99999;
        localCost = 99999;
        isVisited = false;
        parent = nullptr;
    }

    int x = 0;
    int y = 0;

    int heuristicCost = 99999;
    int localCost = 99999;

    bool isVisited = false;

    Node* parent = nullptr;
};

struct Grid
{
public:
    Grid(const unsigned int mapWidth, const unsigned int mapHeight, const unsigned char* pMap)
    {
        gridNodes = std::vector<Node*>(mapWidth * mapHeight);

        for (int i = 0; i < mapWidth * mapHeight; i++)
        {
            gridNodes[i] = new Node();

            const int y = i == 0 ? 0 : i / mapWidth;

            gridNodes[i]->y = y;
            gridNodes[i]->x = i - (y * mapWidth);
        }

        gridWidth = mapWidth;
        gridHeight = mapHeight;
        gridMap = pMap;
    }

    int getIndexFromPosition(const int posX, const int posY)
    {
        return posY * gridWidth + posX;
    }

    bool CheckIsObstacle(int positionX, int positionY)
    {
        return gridMap[getIndexFromPosition(positionX, positionY)] == '\0' ? true : false;
    }

    bool CheckIsObstacle(Node* node)
    {
        return gridMap[getIndexFromPosition(node->x, node->y)] == '\0' ? true : false;
    }

    bool IsNodeValid(int positionX, int positionY)
    {
        return positionX >= 0 && positionX < gridWidth
            && positionY >= 0 && positionY < gridHeight && !CheckIsObstacle(positionX, positionY);
    }

    Node* getNodeFromPosition(const int posX, const int posY)
    {
        if (IsNodeValid(posX, posY))
        {
            return gridNodes[getIndexFromPosition(posX, posY)];
        }

        return nullptr;
    }

    std::vector<Node*> getNeighbours(Node* currentNode)
    {
        auto* parentNode = currentNode->parent;
        std::vector<Node*> neighbours;

        if (parentNode == nullptr)
        {
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    if (x == 0 && y == 0)
                        continue;

                    if (IsNodeValid(x + currentNode->x, y + currentNode->y))
                    {
                        neighbours.emplace_back(gridNodes[getIndexFromPosition(x + currentNode->x, y + currentNode->y)])
                            ->setPositon(x + currentNode->x, y + currentNode->y);
                    }
                }
            }
        }
        else
        {
            int xDirection = std::clamp(currentNode->x - parentNode->x, -1, 1);
            int yDirection = std::clamp(currentNode->y - parentNode->y, -1, 1);

            if (xDirection != 0 && yDirection != 0)
            {
                bool neighbourUp = IsNodeValid(currentNode->x, currentNode->y + yDirection);
                bool neighbourRight = IsNodeValid(currentNode->x + xDirection, currentNode->y);
                bool neighbourLeft = IsNodeValid(currentNode->x - xDirection, currentNode->y);
                bool neighbourDown = IsNodeValid(currentNode->x, currentNode->y - yDirection);

                if (neighbourUp)
                {
                    neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x, currentNode->y + yDirection)])
                        ->setPositon(currentNode->x, currentNode->y + yDirection);
                }

                if (neighbourRight)
                {
                    neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x + xDirection, currentNode->y)])
                        ->setPositon(currentNode->x + xDirection, currentNode->y);
                }

                if (neighbourUp || neighbourRight)
                {
                    if (IsNodeValid(currentNode->x + xDirection, currentNode->y + yDirection))
                    {
                        neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x + xDirection, currentNode->y + yDirection)])
                            ->setPositon(currentNode->x + xDirection, currentNode->y + yDirection);
                    }
                }

                if (!neighbourLeft && neighbourUp)
                {
                    if (IsNodeValid(currentNode->x - xDirection, currentNode->y + yDirection))
                    {
                        neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x - xDirection, currentNode->y + yDirection)])
                            ->setPositon(currentNode->x - xDirection, currentNode->y + yDirection);
                    }
                }

                if (!neighbourDown && neighbourRight)
                {
                    if (IsNodeValid(currentNode->x + xDirection, currentNode->y - yDirection))
                    {
                        neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x + xDirection, currentNode->y - yDirection)])
                            ->setPositon(currentNode->x + xDirection, currentNode->y - yDirection);
                    }
                }
            }
            else
            {
                if (xDirection == 0)
                {
                    if (IsNodeValid(currentNode->x, currentNode->y + yDirection))
                    {
                        neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x, currentNode->y + yDirection)])
                            ->setPositon(currentNode->x, currentNode->y + yDirection);

                        if (!IsNodeValid(currentNode->x + 1, currentNode->y))
                            if (IsNodeValid(currentNode->x + 1, currentNode->y + yDirection))
                            {
                                neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x + 1, currentNode->y + yDirection)])
                                    ->setPositon(currentNode->x + 1, currentNode->y + yDirection);
                            }

                        if (!IsNodeValid(currentNode->x - 1, currentNode->y))
                            if (IsNodeValid(currentNode->x - 1, currentNode->y + yDirection))
                            {
                                neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x - 1, currentNode->y + yDirection)])
                                    ->setPositon(currentNode->x - 1, currentNode->y + yDirection);
                            }
                    }
                }
                else
                {
                    if (IsNodeValid(currentNode->x + xDirection, currentNode->y))
                    {
                        neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x + xDirection, currentNode->y)])
                            ->setPositon(currentNode->x + xDirection, currentNode->y);

                        if (!IsNodeValid(currentNode->x, currentNode->y + 1))
                        {
                            neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x + xDirection, currentNode->y + 1)])
                                ->setPositon(currentNode->x + xDirection, currentNode->y + 1);
                        }

                        if (!IsNodeValid(currentNode->x, currentNode->y - 1))
                        {
                            neighbours.emplace_back(gridNodes[getIndexFromPosition(currentNode->x + xDirection, currentNode->y - 1)])
                                ->setPositon(currentNode->x + xDirection, currentNode->y - 1);
                        }
                    }
                }
            }
        }

        return neighbours;
    }

private:
    std::vector<Node*> gridNodes;
    unsigned int gridWidth = 0;
    unsigned int gridHeight = 0;
    const unsigned char* gridMap;
};

Node* Jump(Node* currentNode, Node* parentNode, Node* targetNode, int directionX, int directionY, Grid* grid)
{
    if (currentNode == nullptr || grid->CheckIsObstacle(currentNode))
    {
        return nullptr;
    }

    if (currentNode == targetNode)
    {
        return currentNode;
    }

    isForced = false;

    if (directionX != 0 && directionY != 0)
    {
        if ((!grid->IsNodeValid(currentNode->x - directionX, currentNode->y) && grid->IsNodeValid(currentNode->x - directionX, currentNode->y + directionY)) ||
            (!grid->IsNodeValid(currentNode->x, currentNode->y - directionY) && grid->IsNodeValid(currentNode->x + directionX, currentNode->y - directionY)))
        {
            return currentNode;
        }


        Node* nextHorizontalNode = grid->getNodeFromPosition(currentNode->x + directionX, currentNode->y);
        Node* nextVerticalNode = grid->getNodeFromPosition(currentNode->x, currentNode->y + directionY);

        if (nextHorizontalNode == nullptr || nextVerticalNode == nullptr)
        {
            bool found = false;
            if (nextHorizontalNode != nullptr && grid->IsNodeValid(currentNode->x + directionX, currentNode->y + directionY))
            {
                found = true;
            }

            if (nextVerticalNode != nullptr && grid->IsNodeValid(currentNode->x + directionX, currentNode->y + directionY))
            {
                found = true;
            }

            if (!found)
                return nullptr;
        }

        if (Jump(nextHorizontalNode, currentNode, targetNode, directionY, directionX, grid) != nullptr || Jump(nextVerticalNode, currentNode, targetNode, directionY, directionX, grid) != nullptr)
        {
            if (!isForced)
            {
                Node* temp = grid->getNodeFromPosition(currentNode->x + directionX, currentNode->y + directionY);

                return Jump(temp, currentNode, targetNode, directionY, directionX, grid);
            }
            else
            {
                return currentNode;
            }
        }
    }

    if (directionX != 0)
    {
        if ((grid->IsNodeValid(currentNode->x + directionX, currentNode->y + 1) && !grid->IsNodeValid(currentNode->x, currentNode->y + 1)) ||
            (grid->IsNodeValid(currentNode->x + directionX, currentNode->y - 1) && !grid->IsNodeValid(currentNode->x, currentNode->y - 1)))
        {
            isForced = true;
            return currentNode;
        }
    }
    else
    {
        if ((grid->IsNodeValid(currentNode->x + 1, currentNode->y + directionY) && !grid->IsNodeValid(currentNode->x + 1, currentNode->y)) ||
            (grid->IsNodeValid(currentNode->x - 1, currentNode->y + directionY) && !grid->IsNodeValid(currentNode->x - 1, currentNode->y)))
        {
            isForced = true;
            return currentNode;
        }
    }

    Node* nextNode = grid->getNodeFromPosition(currentNode->x + directionX, currentNode->y + directionY);

    return Jump(nextNode, currentNode, targetNode, directionX, directionY, grid);
}

float CalculateManhattanDistance(Node* firstNode, Node* secondNode)
{
    return abs(firstNode->x - secondNode->x) + abs(firstNode->y - secondNode->y);
}

std::list<Node*> findSuccessors(Node* currentNode, Node* targetNode, Grid* grid)
{
    std::list<Node*> successors;


    std::vector<Node*> neighbours = grid->getNeighbours(currentNode);

    for (Node* neighbour : neighbours)
    {
        int dirX = std::clamp(neighbour->x - currentNode->x, -1, 1);
        int dirY = std::clamp(neighbour->y - currentNode->y, -1, 1);

        Node* jumpPoint = Jump(neighbour, currentNode, targetNode, dirX, dirY, grid);

        if (jumpPoint != nullptr)
        {
            successors.emplace_back(jumpPoint);
        }
    }

    return successors;
}


std::vector<Node*> openList;
std::list<Node*> jumpNodes;

bool CompareNodes(Node* firstNode, Node* secondNode)
{
    return firstNode->heuristicCost < secondNode->heuristicCost;
}


int RetracePath(Node* startNode, Node* targetNode, int* pOutBuffer, Grid* grid)
{
    Node* currentNode = targetNode;
    int index = 0;

    while (currentNode != startNode)
    {
        pOutBuffer[index] = grid->getIndexFromPosition(currentNode->x, currentNode->y);
        currentNode = currentNode->parent;

        if (currentNode == nullptr || currentNode->parent == nullptr)
        {
            return -1;
        }
    }

    return index;
}

void JumpPointSearch(Node* startNode, Node* endNode, Grid* grid)
{
    Node* currentNode = startNode;
    bool found = false;

    currentNode->x = startNode->x;
    currentNode->y = startNode->y;
    currentNode->parent = currentNode;
    openList.push_back(currentNode);
    currentNode->heuristicCost = CalculateManhattanDistance(currentNode, endNode);
    currentNode->localCost = 0;

    while (!openList.empty())
    {
        std::sort(openList.begin(), openList.end(), CompareNodes);
        currentNode = openList.front();
        openList.erase(openList.begin());

        if (currentNode == endNode)
        {
            found = true;

            break;
        }

        currentNode->isVisited = true;

        std::list<Node*> successorNodes = findSuccessors(currentNode, endNode, grid);

        for (Node* successor : successorNodes)
        {
            jumpNodes.emplace_back(successor);

            if (successor->isVisited)
            {
                continue;
            }

            int globalCost = currentNode->localCost + CalculateManhattanDistance(currentNode, successor);

            if (globalCost < successor->localCost)
            {
                successor->localCost = globalCost;
                successor->heuristicCost = CalculateManhattanDistance(successor, endNode);
                successor->parent = currentNode;
                openList.emplace_back(successor);
            }
        }
    }
}

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap,
    const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize)
{
    Grid* grid = new Grid(nMapWidth, nMapHeight, pMap);

    Node* startNode = grid->getNodeFromPosition(nStartX, nStartY);
    Node* targetNode = grid->getNodeFromPosition(nTargetX, nTargetY);

    JumpPointSearch(startNode, targetNode, grid);

    return RetracePath(startNode, targetNode, pOutBuffer, grid);
}

int main()
{
    unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
    int pOutBuffer[12];

    const int counter = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);

    printf("%d \n", counter);

    for (int i = 0; i < counter; i++)
    {
        printf("%d, ", pOutBuffer[i]);
    }


    return 1;
}
