#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_FRAME
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_FRAME

#include <string>

namespace frame {
const std::string kBaseLink{"base_link"};
const std::string kFixBase{"fix_base"};
const std::string kThetaLink{"theta_link"};
const std::string kZLink{"z_link"};
const std::string kRLink{"r_link"};
const std::string kPhiLink{"phi_link"};
const std::string kBellowsTop{"bellows_top"};
const std::string kBellowsRight{"bellows_right"};
const std::string kBellowsLeft{"bellows_left"};

const std::string kDepart{"depart"};

namespace pick {
const int kNum{6};
const std::string kShare1{"pick_share_1"};
const std::string kShare2{"pick_share_2"};
const std::string k1st{"pick_our_1"};
const std::string k2nd{"pick_our_2"};
const std::string k3rd{"pick_our_3"};
const std::string k4th{"pick_our_4"};
const std::string k5th{"pick_our_5"};
}  // namespace pick


namespace place {
const int kNum{6};
const std::string kShare{"place_share"};
const std::string k1st{"place_our_1"};
const std::string k2nd{"place_our_2"};
const std::string k3rd{"place_our_3"};
const std::string k4th{"place_our_4"};
const std::string k5th{"place_our_5"};
}  // namespace place

}  // namespace frame

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_FRAME */
