/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   get_next_line_utils.c                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2022/01/07 15:50:42 by youskim           #+#    #+#             */
/*   Updated: 2022/04/09 13:21:26 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "get_next_line.h"

int	null_strlen(char *s1)
{
	int	length;

	if (s1 == NULL)
		return (0);
	length = 0;
	while (s1[length] != '\0')
		length++;
	return (length);
}

char	*str_cut(char *str)
{
	int		i;
	int		k;
	char	*cut;

	i = 0;
	while (str[i] != '\n')
		i++;
	cut = (char *)malloc(sizeof(char) * (null_strlen(str) - i));
	if (cut == 0)
		return (NULL);
	k = 0;
	i++;
	while (str[i] != '\0')
	{
		cut[k] = str[i];
		k++;
		i++;
	}
	cut[k] = '\0';
	return (cut);
}

char	*str_join(char *s1, char *s2)
{
	int		index1;
	int		index2;
	char	*join_1_2;
	int		s1_len;
	int		s2_len;

	index1 = -1;
	index2 = 0;
	s1_len = null_strlen(s1);
	s2_len = null_strlen(s2);
	join_1_2 = (char *)malloc(sizeof(char) * (s1_len + s2_len + 1));
	if (join_1_2 == 0)
		return (NULL);
	while (++index1 < s1_len)
		join_1_2[index1] = s1[index1];
	while (index2 < s2_len)
	{
		join_1_2[index1] = s2[index2];
		index1++;
		index2++;
	}
	join_1_2[index1] = '\0';
	return (join_1_2);
}

char	*str_cpy(char *s1)
{
	int		i;
	char	*arr;
	int		k;

	i = 0;
	while (s1[i] != '\n')
		i++;
	arr = (char *)malloc(sizeof(char) * (i + 2));
	if (arr == 0)
		return (NULL);
	k = 0;
	while (k <= i)
	{
		arr[k] = s1[k];
		k++;
	}
	arr[k] = '\0';
	return (arr);
}

int	gnl_strchr(char *str, char c)
{
	int		i;

	i = 0;
	while (str[i] != '\0')
	{
		if (str[i] == c)
			return (1);
		i++;
	}
	return (0);
}
