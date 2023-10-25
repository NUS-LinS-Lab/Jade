#ifndef CONTACTINFOONABODY_HPP_
#define CONTACTINFOONABODY_HPP_

namespace dart {
namespace constraint {

struct ContactInfoOnABody
{
  // Index of the contact among all contacts. Starts from 0.
  size_t index;
  // Spatial Jacobian of that contact
  Eigen::MatrixXd spatialJacobian;
};

}  // namespace constraint
}  // namespace dart



#endif // CONTACTINFOONABODY_HPP_
