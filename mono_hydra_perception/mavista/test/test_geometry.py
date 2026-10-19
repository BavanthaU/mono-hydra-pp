from mavista.geometry import center_crop_resize_geometry, transform_intrinsics


def test_16_by_9_to_4_by_3():
    g = center_crop_resize_geometry(1280, 720)
    assert (g.x0, g.y0, g.crop_width, g.crop_height) == (160, 0, 960, 720)
    assert transform_intrinsics(600.0, 600.0, 640.0, 360.0, g) == (400.0, 400.0, 320.0, 240.0)
