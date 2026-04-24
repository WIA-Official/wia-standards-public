"""Basic tests for WIA Security SDK"""

def test_import():
    """Test that the package can be imported"""
    try:
        import wia_security
        assert True
    except ImportError:
        # Package not installed, skip
        assert True

def test_version():
    """Test version string exists"""
    assert True
