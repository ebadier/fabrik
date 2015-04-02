using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

[System.Serializable]
public class IKEffector
{
	/// <summary>
	/// IK Chain End.
	/// </summary>
	public Transform EndEffector;

	/// <summary>
	/// The target constraining the EndEffector.
	/// </summary>
	public Transform IKTarget;
}

public class Fabrik : MonoBehaviour 
{		
	/*Constant object maintaining the relation between joints of a skeleton
	 * iterating through the transforms in a depth-first order.
	 * */
	public class JointInfo
	{
		public readonly float distanceToParent_; // if 0, no parent
		public readonly float distanceToRoot_; // if 0, no parent
		public readonly int id_; // unique id
		public readonly JointInfo parent_;
		public readonly JointInfo fork_; // closest parent with several children
		public readonly JointInfo[] children_;
		public readonly JointInfo[] effectors_; // effectors associated with the chain starting at "this"
		
		public JointInfo(Transform joint, ref int id, ref IKEffector[] definedEffectors) : this(joint, ref id, ref definedEffectors, null, null)
		{
			// NOTHING	
		}
					
		private JointInfo(Transform joint, ref int id, ref IKEffector[] definedEffectors, JointInfo parent, JointInfo fork)
		{
			id_ = id;
			fork_ = fork;
			parent_ = parent;
			if(parent == null)
			{
				distanceToParent_ = 0;
				distanceToRoot_ = 0;
			}
			else
			{
				distanceToParent_ = Vector3.Distance(joint.parent.position, joint.position);
				distanceToRoot_ = parent.distanceToRoot_ + distanceToParent_;
			}
			int nbChildren = joint.childCount;
			if(nbChildren == 0 || (definedEffectors.Count(ef => ef.EndEffector == joint) > 0) ) // joint is effector
			{
				children_ = new JointInfo[0];
				effectors_ = new JointInfo[1];
				effectors_[0] = this;
			}
			else
			{
				children_ = new JointInfo[nbChildren];
				int nbEffectors_ = 0;
				JointInfo childFork = fork_;
				if(nbChildren > 1)
				{
					childFork = this;
				}
				for(int i=0; i < nbChildren; ++i)
				{
					// id is updated with a depth-first iteration
					++id;
					JointInfo jInfo = new JointInfo(joint.GetChild(i), ref id, ref definedEffectors, this, childFork);
					children_[i] = jInfo;
					nbEffectors_ += jInfo.effectors_.Length;
				}
				effectors_ = new JointInfo[nbEffectors_];
				int j = 0;
				foreach(JointInfo jInfo in children_)
				{
					foreach(JointInfo effector in jInfo.effectors_)
					{
						effectors_[j++] = effector;
					}
				}
			}
		}

		public bool Equals(JointInfo jInfo)
	    {
	        return jInfo != null && id_ == jInfo.id_;
	    }
	}
	
	// exposed attributes
	public Transform IKChainRoot; // kinematic chain's root on which IK will be performed
	public IKEffector[] DefinedEffectors;
	//public Transform[] targets; // ordered list of targets
	/*public float treshold;*/ 
	// End exposed attributes
	
	private JointInfo jointInfo_; // constant
	private Transform[] transforms_; // transforms associated with our Chain
	
	void Start()
	{
		int id = 0;
		jointInfo_ = new JointInfo(IKChainRoot, ref id, ref DefinedEffectors);
		transforms_ = new Transform[id+1];
		id = 0;
		InitTransform(IKChainRoot, ref id);
	}
	
	// indexes transforms of interest
	private void InitTransform(Transform transform, ref int id)
	{
		transforms_[id] = transform;
		if (DefinedEffectors.Count(ef => ef.EndEffector == transform) > 0)
		{
			return;
		}

		for (int i = 0; i < transform.childCount; ++i)
		{
			++id;
			InitTransform(transform.GetChild(i), ref id);
		}
	}
	
	void Update()
	{
		Vector3 rootPos = IKChainRoot.position;
		Vector3[] positionTargets = new Vector3[DefinedEffectors.Length];
		for(int i = 0; i < DefinedEffectors.Length; ++i)
		{
			positionTargets[i] = DefinedEffectors[i].IKTarget.position;
		}
		ForwardStep(jointInfo_.effectors_, positionTargets);
		IKChainRoot.position = rootPos;
		BackwardStep(jointInfo_);
	}
	
	/*
	 * Structure allowing to compute the Centroid between different points
	 * TODO : Target prioritization ?
	 */
	private class TargetCentroid
	{
		private int nbMatches_;
		private Vector3 target_;
		public TargetCentroid()
		{
			nbMatches_ = 0;
			target_ = new Vector3(0,0,0);
		}
		
		public void addTarget(Vector3 target)
		{
			target_ += target;
			++nbMatches_;
		}
		
		public Vector3 Target
		{
			get {return target_ / Mathf.Max(nbMatches_, 1);}
		}
	}
	
	// First step : from every end effectors go up to closest fork
	// At fork, determine centroid position for the targets
	// then go up to the next fork
	private void ForwardStep(JointInfo[] effectors, Vector3[] targets)
	{
		Dictionary<JointInfo, TargetCentroid> centroids = new Dictionary<JointInfo, TargetCentroid>();
		for(int i = 0; i < effectors.Length; ++i)
		{
			JointInfo effector = effectors[i];
			Vector3 target = targets[i];
			transforms_[effector.id_].position = target;
			Vector3 forkTarget = ForwardStepJoint(effector);
			JointInfo fork = effector.fork_;
			if(fork != null)
			{
				if(!centroids.ContainsKey(fork))
				{
					centroids[fork] = new TargetCentroid();
				}
				centroids[fork].addTarget(forkTarget);
			}
		}
		int nbCentroids = centroids.Count;
		if(nbCentroids > 0)
		{
			JointInfo[] upEffectors = new JointInfo[nbCentroids];
			Vector3[] upTargets = new Vector3[nbCentroids];
			int j = 0;
			foreach(KeyValuePair<JointInfo, TargetCentroid> pair in centroids)
			{
				upEffectors[j] = pair.Key;
				upTargets[j]   = pair.Value.Target;
				++j;
			}
			ForwardStep(upEffectors, upTargets);
		}
	}
	
	private Vector3 ForwardStepJoint(JointInfo jointInfo)
	{
		if(jointInfo.parent_ != null)
		{
			Transform transform = transforms_[jointInfo.id_];
			Transform parentTransform = transforms_[jointInfo.parent_.id_];
			float r = Vector3.Distance(transform.position, parentTransform.position);
			float delta = jointInfo.distanceToParent_ / r;
			Vector3 newPos =(1 - delta) *  transform.position + delta * parentTransform.position;
			if(jointInfo.parent_.Equals(jointInfo.fork_)) // parent is fork don't modify position, we'll take centroid
			{
				return newPos;
			}
			else
			{
				MoveTransformToPosition(jointInfo.parent_, newPos);
				return ForwardStepJoint(jointInfo.parent_);
			}
		}
		else
		{
			return IKChainRoot.position;
		}
	}
	
	private void BackwardStep(JointInfo jointInfo)
	{
		if(jointInfo.children_.Length != 0)
		{
			Transform transform = transforms_[jointInfo.id_];
			foreach(JointInfo childInfo in jointInfo.children_)
			{
				Transform childTransform = transforms_[childInfo.id_];
				float r = Vector3.Distance(transform.position, childTransform.position);
				float delta = childInfo.distanceToParent_ / r;
				Vector3 newPos =(1 - delta) * transform.position+ delta * childTransform.position;
				MoveTransformToPosition(childInfo, newPos);
				BackwardStep(childInfo);
			}
		}
	}
	
	private void MoveTransformToPosition(JointInfo jointInfo, Vector3 target)
	{
		Transform transform = transforms_[jointInfo.id_];
		if(jointInfo.parent_ != null) transform.position = target;
	}
}
