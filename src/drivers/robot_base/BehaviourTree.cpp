#include "BehaviourTree.h"
#include <iostream>
#include <list>

class Node
{
public:
	virtual bool willRun() = 0;
};

class CompositeNode : public Node
{
private:
	std::list<Node*> children;

public:
	const std::list<Node*>& getChildren() const
	{
		return children;
	}

	void addChild(Node* child)
	{
		children.emplace_back(child);
	}
};


class Selector : public CompositeNode
{
public:
	virtual bool willRun() override
	{
		for (Node* child : getChildren())
		{
			if (child->willRun())
			{
				//if any child runs the selector succeeds
				return true;
			}
		}
		return false;
	}
};


class Sequence : public CompositeNode
{
public:
	virtual bool willRun() override
	{
		for (Node* child : getChildren())
		{
			if (!child->willRun())
			{
				return false;
			}
		}
		//all children need to succeed for sequence to succeed
		return true;
	}
};

struct CarStatus
{
	//insert needed things into here. not sure whats needed yet.
	//distance to next corner?
	//section of the course?
	//currentspeed?
	//currentgear?
	//etc..
};

//need to make all the tasks under here as well but not 100% sure what they are just yet

//as for speed of the car we could make it based on distance to turn as well as
//how sharp the corner is so the numbers we get back from the code that was on the website
//which makes the car actually turn?


//the car cant do corners very fast unless you implement some op turning
//so just make the car do no more than 60 ish for every single corner
