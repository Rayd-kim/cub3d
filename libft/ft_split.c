/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_split.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 16:49:53 by youskim           #+#    #+#             */
/*   Updated: 2022/05/17 17:35:49 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static int	check_count(char const *s, char c)
{
	size_t	i;
	int		n;

	i = 0;
	n = 0;
	while (i < ft_strlen(s))
	{
		if (s[i] == c)
			i++;
		else
		{
			n++;
			while (s[i] != c && s[i] != '\0')
				i++;
		}
	}
	return (n);
}

static char	*cut_str(char const *s, int len)
{
	char	*cut;
	int		i;

	cut = (char *)malloc(sizeof(char) * (len + 1));
	if (cut == 0)
		return (NULL);
	i = 0;
	while (i < len)
	{
		cut[i] = s[i];
		i++;
	}
	cut[i] = '\0';
	return (cut);
}

static char	**ft_free(char **arr, int count)
{
	while (count > 0)
	{
		free(arr[count - 1]);
		count--;
	}
	return (NULL);
}

static char	**ft_split_result(char **arr, char const *s, char c, int count)
{
	int	i;
	int	k;
	int	cycle;

	i = 0;
	k = 0;
	while (i < count)
	{
		while (s[k] == c)
			k++;
		cycle = 0;
		while (s[k] != c && s[k] != '\0')
		{
			k++;
			cycle++;
		}
		arr[i] = cut_str(&s[k - cycle], cycle);
		if (arr[i] == NULL)
			return (ft_free(arr, i));
		i++;
	}
	arr[i] = NULL;
	return (arr);
}

char	**ft_split(char const *s, char c)
{
	char	**arr;
	int		count;

	if (s == NULL)
		return (NULL);
	count = check_count(s, c);
	arr = (char **)malloc(sizeof(char *) * (count + 1));
	if (arr == 0)
		return (NULL);
	return (ft_split_result(arr, s, c, count));
}
