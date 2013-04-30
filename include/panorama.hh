// File: panorama.hh
// Date: Tue Apr 30 02:02:12 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <memory>
#include "image.hh"
#include "feature.hh"
#include "matrix.hh"

class Panorama {
	private:
		std::vector<imgptr> imgs;

		imgptr get_trans() const;

		void straighten(std::vector<Matrix>& mat) const;

		void straighten_first_last(std::vector<Matrix>& mat) const;

		static Vec2D line_fit(const std::vector<Vec2D>&);

		static Matrix shift_to_line(const std::vector<Vec2D>&, const Vec2D&);

	public:
		Panorama(const std::vector<imgptr>& i) { imgs = i; }

		imgptr get() const;

		static Matrix get_transform(const std::vector<Feature>&, const std::vector<Feature>&); // second -> first

		static std::vector<Feature> get_feature(imgptr);
};