//
// Created by alex on 7/26/25.
//

#ifndef TORCH_NORMAL_H
#define TORCH_NORMAL_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/statistics/distributions/torch_distribution.h"

#include <torch/torch.h>

namespace bitrl
{
    namespace maths::stats
    {
        ///
        /// \brief Normal distribution to be used with torchlib
        ///
        class TorchNormalDist final : public TorchDistributionBase
        {

        public:

            ///
            /// \brief Default constructor
            ///
            ///
            explicit TorchNormalDist(torch_tensor_t mu, torch_tensor_t sigma);

            ///
            /// \brief get a sample from the distribution
            ///
            torch_tensor_t sample()const;


            ///
            /// \brief ~TorchCategorical. Destructor
            ///
            ~TorchNormalDist() override = default;

            ///
            /// \brief log_prob
            /// \param value
            /// \return
            ///
            torch_tensor_t log_prob(torch_tensor_t value) override;

            ///
            ///  \brief Compute the entropy
            /// @return
            torch_tensor_t entropy() const override;

            ///
            /// \brief Returns the mean
            ///
            torch_tensor_t mean()const {return mean_;}

            ///
            /// \brief Returns the standard deviation
            ///
            torch_tensor_t std()const {return sd_;}
        private:

            torch_tensor_t mean_;
            torch_tensor_t sd_;

            ///
            /// \brief Ssample. Sample from the distribution
            /// TODO: Deprecated and should be removed
            ///
            virtual torch_tensor_t sample(c10::ArrayRef<int64_t> /*sample_shape*/) { return torch_tensor_t(); }
        };
    }
}

#endif

#endif //TORCH_NORMAL_H
