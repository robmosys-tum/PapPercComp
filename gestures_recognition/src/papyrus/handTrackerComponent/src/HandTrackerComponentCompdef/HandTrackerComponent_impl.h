// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#ifndef HANDTRACKERCOMPONENTCOMPDEF_HANDTRACKERCOMPONENT_IMPL_H
#define HANDTRACKERCOMPONENTCOMPDEF_HANDTRACKERCOMPONENT_IMPL_H

/************************************************************
 HandTrackerComponent_impl class header
 ************************************************************/

#include "HandTrackerComponentCompdef/Pkg_HandTrackerComponentCompdef.h"

#include "HandTrackerComponentCompdef/HandTrackerComponent.h"

namespace ros2Library {
namespace rclcpp {
class NodeOptions;
}
}

namespace HandTrackerComponentCompdef {

/************************************************************/
/**
 * This is a skeleton class generated for component HandTrackerComponent
 * Copy it into the source folder as an initial base (or copy parts
 * of it whenever you add modify the component).
 * 
 */
class HandTrackerComponent_impl: public HandTrackerComponent {
public:

	/**
	 * 
	 * @param options 
	 */
	HandTrackerComponent_impl(rclcpp::NodeOptions /*in*/options);

	/**
	 * 
	 */
	void videoHandler();

};
/************************************************************/
/* External declarations (package visibility)               */
/************************************************************/

/* Inline functions                                         */

} // of namespace HandTrackerComponentCompdef

/************************************************************
 End of HandTrackerComponent_impl class header
 ************************************************************/

#endif
