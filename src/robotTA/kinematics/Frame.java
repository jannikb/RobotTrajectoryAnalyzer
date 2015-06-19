package robotTA.kinematics;

import java.util.ArrayList;

import urdfMongo.joint.Joint;
import urdfMongo.link.Link;

import Jama.Matrix;

public class Frame {

	private String name;
	private double[] position;
	private double[] rpy;
	private Matrix rotation;
	private Matrix transformation;
	private Joint parentJoint;

	public Frame(Link link) {
		setName(link.getName());
		setPosition(new double[] { 0, 0, 0 });
		setRpy(new double[] { 0, 0, 0 });
		if (link.getParentJoint() != null) {
			setParentJoint(link.getParentJoint());
			setRotation(getRotationMatrix(link.getParentJoint().getRPY()));
			setStaticTransformation(getTransformationMatrix(getRotation(), link
					.getParentJoint().getXYZ()));
		} else {
			setRotation(Matrix.identity(3, 3)); 
			setStaticTransformation(Matrix.identity(4, 4));
		}
	}

	private void setParentJoint(Joint parentJoint) {
		this.parentJoint = parentJoint;
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public double[] getPosition() {
		return position;
	}

	public void setPosition(double[] position) {
		this.position = position;
	}

	public Matrix getRotation() {
		return rotation;
	}

	public void setRotation(Matrix rotation) {
		this.rotation = rotation;
	}

	public double[] getRpy() {
		return rpy;
	}

	public void setRpy(double[] rpy) {
		this.rpy = rpy;
	}

	public Matrix getStaticTransformation() {
		return transformation;
	}

	public void setStaticTransformation(Matrix transformation) {
		this.transformation = transformation;
	}

	private Matrix getRotationMatrix(ArrayList<Double> rpy) {
		double r, p, y, cr, cp, cy, sr, sp, sy;

		r = rpy.get(0);
		p = rpy.get(1);
		y = rpy.get(2);

		cr = Math.cos(r);
		cp = Math.cos(p);
		cy = Math.cos(y);
		sr = Math.sin(r);
		sp = Math.sin(p);
		sy = Math.sin(y);

		return new Matrix(new double[][] {
				{ cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
				{ sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
				{ -sp, cp * sr, cp * cr } });
	}

	private Matrix getTransformationMatrix(Matrix rotation,
			ArrayList<Double> xyz) {
		double[][] transArr = new double[4][4];
		for (int i = 0; i < rotation.getArray().length; i++) {
			for (int j = 0; j < rotation.getArray()[0].length; j++) {
				transArr[i][j] = rotation.get(i, j);
			}
		}

		for (int i = 0; i < xyz.size(); i++) {
			transArr[i][3] = xyz.get(i);
		}

		transArr[3][0] = 0;
		transArr[3][1] = 0;
		transArr[3][2] = 0;
		transArr[3][3] = 1;

		Matrix transMat = new Matrix(transArr);

		return transMat;
	}

	public Matrix getJointPosition(double pos) {

		if (parentJoint == null || pos == 0) {
			return Matrix.identity(4, 4);
		} else {
			Matrix mat;
			switch (parentJoint.getJointType()) {
			case REVOLUTE:
			case CONTINUOUS:
				double[] axis = new double[] { parentJoint.getAxis().get(0),
						parentJoint.getAxis().get(1),
						parentJoint.getAxis().get(2) };
				mat = rotateAboutAxis(axis, pos);
				break;
			case PRISMATIC:
				double[][] arr = new double[4][4];
				arr[0][3] = pos;
				arr[1][3] = pos;
				arr[2][3] = pos;
				arr[3][3] = 1;
				mat = new Matrix(arr);
				break;
			default:
				mat = Matrix.identity(4, 4);
				break;
			}

			return mat;
		}
	}

	public static Matrix rotateAboutAxis(double[] axis, double angle) {
		double u, v, w, u2, v2, w2, ca, sa;

		u = axis[0];
		v = axis[1];
		w = axis[2];
		u2 = u * u;
		v2 = v * v;
		w2 = w * w;

		ca = Math.cos(angle);
		sa = Math.sin(angle);

		return new Matrix(new double[][] {
				{ u2 + (1 - u2) * ca, u * v * (1 - ca) + w * sa,
						u * w * (1 - ca) - v * sa, 0 },
				{ u * v * (1 - ca) + w * sa, v2 + (1 - v2) * ca,
						v * w * (1 - ca) - u * sa, 0 },
				{ u * w * (1 - ca) - v * sa, v * w * (1 - ca) + u * sa,
						w2 + (1 - w2) * ca, 0 }, { 0, 0, 0, 1 } });
	}

}
