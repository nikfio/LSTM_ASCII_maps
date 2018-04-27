



namespace GlobalPlanning {

	Node::Node() 
	{
		pix.x = 0;
		pix.y = 0;
		G = 0; H = 0;
		parent = nullptr;
	};


	Node::Node(Pixel new_pix)
	{
		pix.x = new_pix.x;
		pix.y = new_pix.y;
		parent = nullptr;
		G = 0; H = 0;
	};

	Node::Node(Pixel coordinates_, Node* parent_)
	{
		parent = parent_;
		pix = coordinates_;
		G = 0; H = 0;
	   
	};

	double Node::getAstarScore()
	{
		return G + H;
	};

	const Node Node::operator = (const Node& equal)
	{
		pix = equal.pix;
		G = equal.G;
		H = equal.H;
		sym = equal.sym;
		parent = equal.parent;
	};

	bool Node::operator == (const Node& curr)
	{
		if( pix==curr.pix && G==curr.G && H==curr.H && parent==curr.parent)
			return true;
		else
			return false;

	}
