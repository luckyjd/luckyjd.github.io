# Install Ruby
$ sudo apt-get update
$ sudo apt-get install ruby-full build-essential zlib1g-dev
$ ruby -v    # check 

$ sudo gem install bundler  # can find other way , without sudo
$ cd path/to/your/project
$ sudo bundle install

# check directory gem
$ bundle show jekyll-theme-leap-day


# build 
$ bundle exec jekyll clean

$ bundle exec jekyll serve
