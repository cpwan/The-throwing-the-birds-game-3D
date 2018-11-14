#ifndef BIRDSUBSIM_H
#define BIRDSUBSIM_H

#include "SubSim.h"

/*
 * 
 */


struct Spring {
   public:
    double stiffness;
    double damping;
};


class BirdSubSim : public SubSim {
public:
	BirdSubSim(CommonSim* parent);

	virtual void addObjects() override;

	virtual void resetMembers() override;
	virtual bool advance(float time, float dt) override;

	virtual void drawSimulationParameterMenu() override;



#pragma region SettersAndGetters

	void setCustom(double custom);

#pragma endregion SettersAndGetters

private:
	double m_custom;

	double m_angle;
	double m_initial_impluse;
	int m_leftFoot;
	int m_rightFoot;
	int m_body;


	RigidObject *body;
	RigidObject *leftFoot;
	RigidObject *rightFoot;

	double m_mass;
	double m_floor_stiffness;
	double m_floor_friction;
	Eigen::Vector3d m_gravity;

	Spring m_spring;
	double spring_scale;
	std::vector<double> m_lengths;
	std::vector<Eigen::Vector3d> m_velocities;
	Eigen::MatrixXi m_edges;

	Eigen::MatrixXd m_renderV, m_Vorig;  // vertex positions for rendering
	Eigen::MatrixXi m_renderF, m_Forig;  // face indices for rendering


	Eigen::MatrixXd m_Vorig_init;  
	Eigen::MatrixXi m_Forig_init; 
	Eigen::MatrixXd m_Vorig_rigid;
	Eigen::MatrixXi m_Forig_rigid;
};

#endif