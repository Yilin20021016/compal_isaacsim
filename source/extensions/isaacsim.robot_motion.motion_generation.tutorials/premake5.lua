-- Setup the basic extension variables
local ext = get_current_extension_info()
-- Set up the basic shared project information
project_ext (ext)

-- -------------------------------------
-- Link/copy folders and files to be packaged with the extension
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
    { "isaacsim", ext.target_dir.."/isaacsim" },
}
